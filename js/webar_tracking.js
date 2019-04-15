(function(){
  var WebArTracking = (function(){

    MIN_TIMESTEP = 0.001;
    MAX_TIMESTEP = 1;
    var DEBUG = false;


    // //these are fine-tuning parameters that have not yet been fine-tuned.
    // Q_POSITION = 0.1;
    // Q_VELOCITY = 0.1;


    // R_POSITION_MULTIPLIER = 1;//multiplied by geolocation accuracy to get accuracy of gps
    // R_VELOCITY = 0.1;//not sure if this has measurable effect


    // REQUEST_HIGH_ACCURACY = true;
    // TIME_BETWEEN_REQUESTS = 200;//this is in ms.

    // Helper method to validate the time steps of sensor timestamps.
    function isTimestampDeltaValid(timestampDeltaS) {
      if (isNaN(timestampDeltaS)) {
        return false;
      }
      if (timestampDeltaS <= MIN_TIMESTEP) {
        return false;
      }
      if (timestampDeltaS > MAX_TIMESTEP) {
        return false;
      }
      return true;
    }


    function SensorSample(sample, timestampS) {
      this.set(sample, timestampS);
    };

    SensorSample.prototype.set = function(sample, timestampS) {
      this.sample = sample;
      this.timestampS = timestampS;
    };

    SensorSample.prototype.copy = function(sensorSample) {
      this.set(sensorSample.sample, sensorSample.timestampS);
    };




    function KalmanFilter() {
      this.x = $V([1,0]);
      this.P = $M([
        [Q_POSITION,0],
        [0,Q_VELOCITY]
      ]);
    }
    KalmanFilter.prototype.updatepos = function(position,accuracy) {
      var H = $M([
        [1, 0],
        [0, 1]
      ]);
      var R = $M([
        [accuracy*R_POSITION_MULTIPLIER,0],
        [0,R_VELOCITY]
      ]);

      var measurement = $V([position,0]);

      // correction
      var S = ((H.multiply(this.P)).multiply(H.transpose())).add(R);
      var K = (this.P.multiply(H.transpose())).multiply(S.inverse());
      var y = measurement.subtract(H.multiply(this.x));


      this.x = this.x.add(K.multiply(y));
      this.P = ((Matrix.I(2)).subtract(K.multiply(H))).multiply(this.P);
    };
    KalmanFilter.prototype.updateacc = function(accelleration,time) {
      var A = $M([
        [1,time],
        [0, 1]
      ]);
      var B = $M([
        [0, 0],
        [time,0]
      ]);
      var Q = $M([
        [Q_POSITION,0],
        [0,Q_VELOCITY]
      ]);

      var control = $V([accelleration, 0]);
      // prediction
      this.x = (A.multiply(this.x)).add(B.multiply(control));
      this.P = ((A.multiply(this.P)).multiply(A.transpose())).add(Q); 
    };

    KalmanFilter.prototype.getestimate = function() {
      return this.x.elements[0];
    };
    //{enableHighAccuracy: true, distanceFilter: 1, timeout: 1000}



    function FixedKalFilter(kFilter) {
      this.kFilter = kFilter;


      // this.currentAccelWorldspace = new SensorSample();
      // this.estimatedPosition = new THREE.Vector2(0,0);

      // this.xfilter = new KalmanFilter();
      // this.yfilter = new KalmanFilter();




      // Raw sensor measurements.
      this.currentAccelMeasurement = new SensorSample();
      this.currentGyroMeasurement = new SensorSample();
      this.previousGyroMeasurement = new SensorSample();

      // Current filter orientation
      this.filterQ = new THREE.Quaternion();
      this.previousFilterQ = new THREE.Quaternion();

      // Orientation based on the accelerometer.
      this.accelQ = new THREE.Quaternion();
      // Whether or not the orientation has been initialized.
      this.isOrientationInitialized = false;
      this.isGPSInitialized = false;
      // Running estimate of gravity based on the current orientation.
      this.estimatedGravity = new THREE.Vector3();
      // Measured gravity based on accelerometer.
      this.measuredGravity = new THREE.Vector3();
      // Debug only quaternion of gyro-based orientation.
      // this.gyroIntegralQ = new THREE.Quaternion();
    }

    // FixedKalFilter.prototype.addGPSMeasurement = function(coords,timestampS) {
    //   if (!this.isGPSInitialized) {
    //     this.isGPSInitialized = true;
    //     this.gpsReferenceFrame = coords;
    //   }
    //   this.xfilter.updatepos((coords.longitude-this.gpsReferenceFrame.longitude)*111194.926517,coords.accuracy);
    //   this.yfilter.updatepos((coords.latitude-this.gpsReferenceFrame.latitude)*111194.926517,coords.accuracy);
    // };

    FixedKalFilter.prototype.addAccelMeasurement = function(vector,timestampS) {
      this.currentAccelMeasurement.set(vector, timestampS);
      // this.currentAccelWorldspace.set(nogravity, timestampS);
    };

    FixedKalFilter.prototype.addGyroMeasurement = function(vector, timestampS) {
      this.currentGyroMeasurement.set(vector, timestampS);

      var deltaT = timestampS - this.previousGyroMeasurement.timestampS;
      if (isTimestampDeltaValid(deltaT)) {
        this.run_();
      }
      
      this.previousGyroMeasurement.copy(this.currentGyroMeasurement);
    };

    FixedKalFilter.prototype.run_ = function() {
      this.accelQ = this.accelToQuaternion_(this.currentAccelMeasurement.sample);

      if (!this.isOrientationInitialized) {
        this.previousFilterQ.copy(this.accelQ);
        this.isOrientationInitialized = true;
        return;
      }
      var deltaT = this.currentGyroMeasurement.timestampS -
          this.previousGyroMeasurement.timestampS;

      // Convert gyro rotation vector to a quaternion delta.
      var gyroDeltaQ = this.gyroToQuaternionDelta_(this.currentGyroMeasurement.sample, deltaT);
      // this.gyroIntegralQ.multiply(gyroDeltaQ);

      // filter_1 = K * (filter_0 + gyro * dT) + (1 - K) * accel.
      this.filterQ.copy(this.previousFilterQ);
      this.filterQ.multiply(gyroDeltaQ);

      // Calculate the delta between the current estimated gravity and the real
      // gravity vector from accelerometer.
      var invFilterQ = new THREE.Quaternion();
      invFilterQ.copy(this.filterQ);
      invFilterQ.inverse();

      this.estimatedGravity.set(0, 0, -1);
      this.estimatedGravity.applyQuaternion(invFilterQ);
      this.estimatedGravity.normalize();

      this.measuredGravity.copy(this.currentAccelMeasurement.sample);
      this.measuredGravity.normalize();

      // Compare estimated gravity with measured gravity, get the delta quaternion
      // between the two.
      var deltaQ = new THREE.Quaternion();
      deltaQ.setFromUnitVectors(this.estimatedGravity, this.measuredGravity);
      deltaQ.inverse();

      // Calculate the SLERP target: current orientation plus the measured-estimated
      // quaternion delta.
      var targetQ = new THREE.Quaternion();
      targetQ.copy(this.filterQ);
      targetQ.multiply(deltaQ);

      // SLERP factor: 0 is pure gyro, 1 is pure accel.
      this.filterQ.slerp(targetQ, 1 - this.kFilter);

      this.previousFilterQ.copy(this.filterQ);



      // if (this.isGPSInitialized) {
      //   this.xfilter.updateacc(this.currentAccelWorldspace.sample.x,deltaT);
      //   this.yfilter.updateacc(this.currentAccelWorldspace.sample.y,deltaT);
      //   this.estimatedPosition.set(this.xfilter.getestimate(),this.yfilter.getestimate());
      // }


    };

    FixedKalFilter.prototype.getOrientation = function() {
      return this.filterQ;
    };
    // FixedKalFilter.prototype.getPosition = function() {
    //   return this.estimatedPosition;
    // };

    FixedKalFilter.prototype.accelToQuaternion_ = function(accel) {
      var normAccel = new THREE.Vector3();
      normAccel.copy(accel);
      normAccel.normalize();
      var quat = new THREE.Quaternion();
      quat.setFromUnitVectors(new THREE.Vector3(0, 0, -1), normAccel);
      return quat;
    };

    FixedKalFilter.prototype.gyroToQuaternionDelta_ = function(gyro, dt) {
      // Extract axis and angle from the gyroscope data.
      var quat = new THREE.Quaternion();
      var axis = new THREE.Vector3();
      axis.copy(gyro);
      axis.normalize();
      quat.setFromAxisAngle(axis, gyro.length() * dt);
      return quat;
    };







    function OrientationPredictor(predictionTimeS) {
      this.predictionTimeS = predictionTimeS;

      // The quaternion corresponding to the previous state.
      this.previousQ = new THREE.Quaternion();
      // Previous time a prediction occurred.
      this.previousTimestampS = null;

      // The delta quaternion that adjusts the current pose.
      this.deltaQ = new THREE.Quaternion();
      // The output quaternion.
      this.outQ = new THREE.Quaternion();
    }

    OrientationPredictor.prototype.getPrediction = function(currentQ, gyro, timestampS) {
      if (!this.previousTimestampS) {
        this.previousQ.copy(currentQ);
        this.previousTimestampS = timestampS;
        return currentQ;
      }

      // Calculate axis and angle based on gyroscope rotation rate data.
      var axis = new THREE.Vector3();
      axis.copy(gyro);
      axis.normalize();

      var angularSpeed = gyro.length();

      // If we're rotating slowly, don't do prediction.
      if (angularSpeed < THREE.Math.degToRad(20)) {
        this.outQ.copy(currentQ);
        this.previousQ.copy(currentQ);
        return this.outQ;
      }

      // Get the predicted angle based on the time delta and latency.
      var deltaT = timestampS - this.previousTimestampS;
      var predictAngle = angularSpeed * this.predictionTimeS;

      this.deltaQ.setFromAxisAngle(axis, predictAngle);
      this.outQ.copy(this.previousQ);
      this.outQ.multiply(this.deltaQ);

      this.previousQ.copy(currentQ);

      return this.outQ;
    };



    function PhoneTracker() {

      this.accelerometer = new THREE.Vector3();
      this.accelerometernogravity = new THREE.Vector3();
      this.gyroscope = new THREE.Vector3();
      this.forcedRefOffset = new THREE.Vector2(0,0);
      // alert("sefasdfa");

      // window.addEventListener('devicemotion', this.onDeviceMotionChange_.bind(this));
      // window.addEventListener('orientationchange', this.onScreenOrientationChange_.bind(this));




      // navigator.geolocation.getCurrentPosition(respond,function(){},{enableHighAccuracy:REQUEST_HIGH_ACCURACY,maximumAge:10,timeout:Infinity});

      // var self=this;
      // function respond(position) {
      //   self.filter.addGPSMeasurement(position.coords);
      //   setTimeout(function() {
      //     navigator.geolocation.getCurrentPosition(respond,function(){},{enableHighAccuracy:REQUEST_HIGH_ACCURACY,maximumAge:10,timeout:Infinity});
      //   },TIME_BETWEEN_REQUESTS);
      // }


      this.filter = new FixedKalFilter(0.98);
      this.posePredictor = new OrientationPredictor(0.050);

      this.filterToWorldQ = new THREE.Quaternion();

      // Set the filter to world transform, but only for Android.
      if (/iPad|iPhone|iPod/.test(navigator.userAgent) && !window.MSStream) {
        this.filterToWorldQ.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI/2);
      } else {
        this.filterToWorldQ.setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI/2);
      }

      this.worldToScreenQ = new THREE.Quaternion();
      this.setScreenTransform_();
    }



    PhoneTracker.prototype.handleDeviceMotionEvent = function(deviceMotion) {
      var accGravity = deviceMotion.accelerationIncludingGravity;
      var acc = deviceMotion.acceleration;
      var rotRate = deviceMotion.rotationRate;
      var timestampS = deviceMotion.timeStamp / 1000;

      this.accelerometer.set(-accGravity.x, -accGravity.y, -accGravity.z);
      this.accelerometernogravity.set(-acc.x, -acc.y, -acc.z);
      this.gyroscope.set(rotRate.alpha, rotRate.beta, rotRate.gamma);

      this.gyroscope.multiplyScalar(Math.PI / 180);

      this.filter.addAccelMeasurement(this.accelerometer,timestampS);
      this.filter.addGyroMeasurement(this.gyroscope, timestampS);

      this.previousTimestampS = timestampS;
    };


    PhoneTracker.prototype.handleScreenOrientationEvent = function(screenOrientation) {
      this.setScreenTransform_();
    };

    PhoneTracker.prototype.setScreenTransform_ = function() {
      this.worldToScreenQ.set(0, 0, 0, 1);
      switch (window.orientation) {
        case 0:
          break;
        case 90:
          this.worldToScreenQ.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI/2);
          break;
        case -90: 
          this.worldToScreenQ.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI/2);
          break;
        case 180:
          break;
      }
    };
    // PhoneTracker.prototype.forceReferenceFrame = function(frame) {
    //   this.forcedReferenceFrame = frame;
    //   this.forcedRefOffset = null;
    // };
    // PhoneTracker.prototype.getReferenceFrame = function() {
    //   return this.filter.gpsReferenceFrame;
    // };
    PhoneTracker.prototype.getOrientationQuaternion = function() {
      // Convert from filter space to the the same system used by the
      // deviceorientation event.
      var orientation = this.filter.getOrientation();
      // Predict orientation.
      this.predictedQ = this.posePredictor.getPrediction(orientation, this.gyroscope, this.previousTimestampS);

      // Convert to THREE coordinate system: -Z forward, Y up, X right.
      var out = new THREE.Quaternion();
      out.copy(this.filterToWorldQ);
      out.multiply(this.predictedQ);
      out.multiply(this.worldToScreenQ);

      //go back to css transform coordinates
      // out.w = out.w*-1;
      // out.z = out.z*-1;
      // out.x = out.x*-1;

      return out;
    };
    PhoneTracker.prototype.getAccelerationWorld = function() {
      var qua = this.getOrientationQuaternion();
      var x = new THREE.Vector3(this.accelerometernogravity.x,this.accelerometernogravity.y,this.accelerometernogravity.z);
      x.applyQuaternion(qua);
      return x;
    };
    PhoneTracker.prototype.getOrientationEuler = function() {
      var rotation = new THREE.Euler().setFromQuaternion(this.getOrientationQuaternion(),'XYZ');
      return rotation;
    };


    // PhoneTracker.prototype.getPosition = function() {
    //   if (this.filter.isGPSInitialized) {
    //     if (this.forcedRefOffset===null) {
    //       this.forcedRefOffset = new THREE.Vector2((this.filter.gpsReferenceFrame.longitude-this.forcedReferenceFrame.longitude)*111194.926517,(this.filter.gpsReferenceFrame.latitude-this.forcedReferenceFrame.latitude)*111194.926517);
    //     }
    //     var posit = this.filter.getPosition().add(this.forcedRefOffset);
    //     console.log(posit);
    //     return posit;
    //   } else {
    //     return null;
    //   }
    // };

      return {
        SensorSample: SensorSample,
        KalmanFilter: KalmanFilter,
        FixedKalFilter: FixedKalFilter,
        OrientationPredictor: OrientationPredictor,
        PhoneTracker: PhoneTracker
      };

    })();

    if (typeof module !== 'undefined' && typeof module.exports !== 'undefined'){
      module.exports = WebArTracking;
    } else {
      window.WebArTracking = WebArTracking;
    }
})();
