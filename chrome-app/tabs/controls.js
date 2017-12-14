/**
* Created by Neha on 25-10-2016.
*/
'use strict';

var refreshIntervalId = null,
rc_array_buffer = [],
i,
h,
controlsCheck = false,
joystickCheck = false,
armVal = 1200,
joystick_array = [],
buffering_set_rc = [],
buffer_delay = false,
activeArm = false,
minVal = 1000,
arm_enable = false;

TABS.controls = {
    yaw_fix: 0.0
};
TABS.controls.initialize = function (callback) {
    var self = this;
    

    
    if (GUI.active_tab != 'controls') {
        GUI.active_tab = 'controls';
        googleAnalytics.sendAppView('Controls');
    }
    
    function load_status() {
        MSP.send_message(MSP_codes.MSP_STATUS, false, false, load_ident);
    }
    
    function load_ident() {
        MSP.send_message(MSP_codes.MSP_IDENT, false, false, load_config);
    }
    
    function load_config() {
        MSP.send_message(MSP_codes.MSP_BF_CONFIG, false, false, load_misc_data);
    }
    
    function load_misc_data() {
        MSP.send_message(MSP_codes.MSP_MISC, false, false, load_html);
    }
    
    function load_html() {
        $('#content').load("./tabs/controls.html", process_html);
    }
    
    MSP.send_message(MSP_codes.MSP_ACC_TRIM, false, false, load_status);
    
    function process_html() {
        // translate to user-selected language
        console.log("processing html");
        localize();

  /* var imported = document.createElement('script');
	imported.src = './tabs/common.js';
	document.head.appendChild(imported);
    */
    
  
  
        // initialize 3D
        self.initialize3D();
        // set roll in interactive block
        $('span.roll').text(chrome.i18n.getMessage('initialSetupAttitude', [0]));
        // set pitch in interactive block
        $('span.pitch').text(chrome.i18n.getMessage('initialSetupAttitude', [0]));
        // set heading in interactive block
        $('span.heading').text(chrome.i18n.getMessage('initialSetupAttitude', [0]));
        
        self.initializeInstruments();
        // UI Hooks
        
        $('a.resetSettings').click(function () {
            MSP.send_message(MSP_codes.MSP_RESET_CONF, false, false, function () {
                GUI.log(chrome.i18n.getMessage('initialSetupSettingsRestored'));
                GUI.tab_switch_cleanup(function () {
                    TABS.controls.initialize();
                });
            });
        });
        // display current yaw fix value (important during tab re-initialization)
        $('div#interactive_block > a.reset').text(chrome.i18n.getMessage('initialSetupButtonResetZaxisValue', [self.yaw_fix]));
        // reset yaw button hook
        $('div#interactive_block > a.reset').click(function () {
            self.yaw_fix = SENSOR_DATA.kinematics[2] * - 1.0;
            $(this).text(chrome.i18n.getMessage('initialSetupButtonResetZaxisValue', [self.yaw_fix]));
            console.log('YAW reset to 0 deg, fix: ' + self.yaw_fix + ' deg');
        });
     
        // cached elements
        var bat_voltage_e = $('.bat-voltage'),
            bat_mah_drawn_e = $('.bat-mah-drawn'),
            bat_mah_drawing_e = $('.bat-mah-drawing'),
            rssi_e = $('.rssi'),
            gpsFix_e = $('.gpsFix'),
            gpsSats_e = $('.gpsSats'),
            gpsLat_e = $('.gpsLat'),
            gpsLon_e = $('.gpsLon'),
            roll_e = $('dd.roll'),
            pitch_e = $('dd.pitch'),
            heading_e = $('dd.heading');
        function get_slow_data() {
  
            MSP.send_message(MSP_codes.MSP_STATUS);
            MSP.send_message(MSP_codes.MSP_ANALOG, false, false, function () {
                bat_voltage_e.text(chrome.i18n.getMessage('initialSetupBatteryValue', [ANALOG.voltage]));
                bat_mah_drawn_e.text(chrome.i18n.getMessage('initialSetupBatteryMahValue', [ANALOG.mAhdrawn]));
                bat_mah_drawing_e.text(chrome.i18n.getMessage('initialSetupBatteryAValue', [ANALOG.amperage.toFixed(2)]));
                rssi_e.text(chrome.i18n.getMessage('initialSetupRSSIValue', [((ANALOG.rssi / 1023) * 100).toFixed(0)]));
            });
            if (have_sensor(CONFIG.activeSensors, 'gps')) {
                MSP.send_message(MSP_codes.MSP_RAW_GPS, false, false, function () {
                    gpsFix_e.html((GPS_DATA.fix) ? chrome.i18n.getMessage('gpsFixTrue') : chrome.i18n.getMessage('gpsFixFalse'));
                    gpsSats_e.text(GPS_DATA.numSat);
                    gpsLat_e.text((GPS_DATA.lat / 10000000).toFixed(4) + ' deg');
                    gpsLon_e.text((GPS_DATA.lon / 10000000).toFixed(4) + ' deg');
                });
            }
        }
        function get_fast_data() {

            MSP.send_message(MSP_codes.MSP_ATTITUDE, false, false, function () {

                roll_e.text(chrome.i18n.getMessage('initialSetupAttitude', [SENSOR_DATA.kinematics[0]]));
                pitch_e.text(chrome.i18n.getMessage('initialSetupAttitude', [SENSOR_DATA.kinematics[1]]));
                heading_e.text(chrome.i18n.getMessage('initialSetupAttitude', [SENSOR_DATA.kinematics[2]]));
                self.render3D();
                self.updateInstruments();
            });
        }
        
  
  
     
    
        $('#enableGamepad').prop('checked', false);
        $('#enableGamepad').prop('disabled', false);  
        
        loadGamepadModule();
        
        	$('#GPStatus').hide();
        	
 			$('#enableGamepad').change(function () {
				 if($(this).is(':checked')){
 
 
 					if(refreshIntervalId!= null){
 					window.clearInterval(refreshIntervalId);
 					console.log("refreshIntervalId", refreshIntervalId);
 					}

                refreshIntervalId = window.setInterval( update, 150);
     	 
				   joystickCheck= true;
   				var statusis = document.getElementById('armField').value;
 	    					
                  function update() {
               
               
                   var armValue= joystick_array[7];
                   if(armValue==1)
                   {
                   	if(arm_enable)
                   	arm_enable= false;
                   	else
                   	arm_enable= true;
                   }
            
                     
                   if(arm_enable){
                   	    
                   var val=1500;
                   var statusis="GAMEPAD ARMED";
                   joystick_array[7]=val;
                   }
                   else {
                   	var statusis="GAMEPAD DISARMED";
                   	var val=1200;
                   	joystick_array[7]=val;
                   }
                   	
                   if(statusis == "GAMEPAD DISARMED"){
   							 armField.innerHTML="GAMEPAD DISARMED";
 						 }
  						 else {
    							armField.innerHTML="GAMEPAD ARMED";
  						}
                   	
            		console.log("gamepad values:"+joystick_array);
                  MSP.setRawRx(joystick_array);
            }
                  
                  
                     console.log(refreshIntervalId);
                     $('#activeGP').css({'background':'#a1f89c'});
 							$('#activeControls').css({'background':'white'});
 			
 							$('#GPStatus').show();
 							$('#ControlsStatus').hide();
 
   }
 else {

     console.log("in else");

     		window.clearInterval(refreshIntervalId);

      	console.log(refreshIntervalId);
      	
			joystick_array[7]=1200;

         armField.innerHTML="GAMEPAD DISARMED";
                    
         console.log(joystick_array);
         MSP.setRawRx(joystick_array); 
         
          $('#activeControls').css({'background':'#a1f89c'});
 			 $('#activeGP').css({'background':'white'});
 			 
 			 $('#GPStatus').hide();
 			 $('#ControlsStatus').show();
        
   
      }  
     
});

		 if (joystickCheck == false) {
 	                                                                              
			if (!($('#enableGamepad').is(':checked'))) {
				activeArm = false;
  		
     	     $('#arm').on('click', function(){
   		  var currentvalue = document.getElementById('arm').value;	
			  if(currentvalue == "DISARMED") {	
     			document.getElementById("arm").value="ARMED";
            activeArm = true;
            $('div.insert input').trigger('input'); 
           }
           else {
            document.getElementById("arm").value="DISARMED";
            activeArm = false;
            $('div.insert input').trigger('input'); 
           }
     
         });

			$('div.insert input').on('input', function () {

                    for(i = 0;i < 4; i++) {	
                  	 var val = parseInt($('div.insert input').eq(i).val());
                  	 joystick_array[i]=val;
                    }
                         
                    for(i=0;i<2;i++) {
    							var jvalue=((joystick_array[i]*25)+1500);
    							joystick_array[i]=jvalue;
    					   }
    							jvalue=((joystick_array[2]*10)+1000);
    							joystick_array[2]=jvalue;
    					   	
    					   	jvalue=((joystick_array[3]*5)+1500);
    					   	joystick_array[3]=jvalue;
    						                  
    						joystick_array[4]=minVal;
                   	joystick_array[5]=minVal;
                   	joystick_array[6]=minVal;
                   	if(activeArm) {              	
                    		joystick_array[7]=1500;
                     }
                     else {
                     	joystick_array[7]=minVal;	
                     }
							
   console.log(joystick_array);
   MSP.setRawRx(joystick_array); 
               
 	});
}
}


$('put[type="number"]').each(function () {
   (this).on('keyup', function () {
        if ($(this).val() > Number($(this).attr("max"))) {
            var val = $(this).val().slice(0, $(this).attr("max").length);
            $(this).val(val);
        }
    });
});
                                                      


        
        GUI.interval_add('setup_data_pull_fast', get_fast_data, 33, true); // 30 fps
        GUI.interval_add('setup_data_pull_slow', get_slow_data, 250, true); // 4 fps
        
      
       
        GUI.content_ready(callback);
  
    }
};

TABS.controls.initializeInstruments = function() {
    var options = {size:90, showBox : false, img_directory: 'images/flightindicators/'};
    var attitude = $.flightIndicator('#attitude', 'attitude', options);
    var heading = $.flightIndicator('#heading', 'heading', options);
    this.updateInstruments = function() {
        attitude.setRoll(SENSOR_DATA.kinematics[0]);
        attitude.setPitch(SENSOR_DATA.kinematics[1]);
        heading.setHeading(SENSOR_DATA.kinematics[2]);
    };
};
TABS.controls.initialize3D = function (compatibility) {

    var self = this,
        loader, canvas, wrapper, renderer, camera, scene, light, light2, modelWrapper, model, model_file,
        useWebGlRenderer = false;
    canvas = $('.model-and-info #canvas');
    wrapper = $('.model-and-info #canvas_wrapper');
    // webgl capability detector
    // it would seem the webgl "enabling" through advanced settings will be ignored in the future
    // and webgl will be supported if gpu supports it by default (canary 40.0.2175.0), keep an eye on this one
    var detector_canvas = document.createElement('canvas');
    if (window.WebGLRenderingContext && (detector_canvas.getContext('webgl') || detector_canvas.getContext('experimental-webgl'))) {
        renderer = new THREE.WebGLRenderer({canvas: canvas.get(0), alpha: true, antialias: true});
        useWebGlRenderer = true;
    } else {
        renderer = new THREE.CanvasRenderer({canvas: canvas.get(0), alpha: true});
    }
    // initialize render size for current canvas size
    renderer.setSize(wrapper.width()*2, wrapper.height()*2);
//    // modelWrapper adds an extra axis of rotation to avoid gimbal lock with the euler angles
    modelWrapper = new THREE.Object3D();
//
    // load the model including materials
    if (useWebGlRenderer) {
        model_file = mixerList[CONFIG.multiType - 1].model;
    } else {
        model_file = 'fallback'
    }
    // Temporary workaround for 'custom' model until akfreak's custom model is merged.
    var useLegacyCustomModel = false;
    if (model_file == 'custom') {
        //  model_file = 'fallback';
        model_file = 'quad_x';
        useLegacyCustomModel = true;
    }
    // setup scene
    scene = new THREE.Scene();
    loader = new THREE.JSONLoader();
    loader.load('./resources/models/' + model_file + '.json', function (geometry, materials) {
        var modelMaterial = new THREE.MeshFaceMaterial(materials);
        model = new THREE.Mesh(geometry, modelMaterial);
        model.scale.set(15, 15, 15);
        modelWrapper.add(model);
        scene.add(modelWrapper);
    });
    // stationary camera
    camera = new THREE.PerspectiveCamera(50, wrapper.width() / wrapper.height(), 1, 10000);
    // some light
    light = new THREE.AmbientLight(0x404040);
    light2 = new THREE.DirectionalLight(new THREE.Color(1, 1, 1), 1.5);
    light2.position.set(0, 1, 0);
    // move camera away from the model
    camera.position.z = 125;
    // add camera, model, light to the foreground scene
    scene.add(light);
    scene.add(light2);
    scene.add(camera);
    scene.add(modelWrapper);
    this.render3D = function () {

        if (!model) {
            return;
        }
        // compute the changes
        model.rotation.x = (SENSOR_DATA.kinematics[1] * -1.0) * 0.017453292519943295;
        modelWrapper.rotation.y = ((SENSOR_DATA.kinematics[2] * -1.0) - self.yaw_fix) * 0.017453292519943295;
        model.rotation.z = (SENSOR_DATA.kinematics[0] * -1.0) * 0.017453292519943295;
        // draw
        renderer.render(scene, camera);
    };
    // handle canvas resize
    this.resize3D = function () {
        renderer.setSize(wrapper.width()*2, wrapper.height()*2);
        camera.aspect = wrapper.width() / wrapper.height();
        camera.updateProjectionMatrix();
        self.render3D();
    };
    $(window).on('resize', this.resize3D);
};

TABS.controls.cleanup = function (callback) {
	  removeGamepadModule();
	 joystick_array[7]=1200;
    MSP.setRawRx(joystick_array); 
	
    
    if(refreshIntervalId!= null) {
    	window.clearInterval(refreshIntervalId);
    }
      console.log("refreshIntervalId afterwards", refreshIntervalId);
    
    $(window).off('resize', this.resize3D);

    if (callback) callback();
};