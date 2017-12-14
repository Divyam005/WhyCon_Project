'use strict';

var serial = {
    connectionId:    false,
    openRequested:   false,
    openCanceled:    false,
    bitrate:         0,
    bytesReceived:   0,
    bytesSent:       0,
    failed:          0,

    transmitting:   false,
    outputBuffer:  [],

    connect: function (path, options, callback) {
        var self = this;
        self.openRequested = true;

        chrome.sockets.tcp.create({}, function(createInfo) {
            chrome.sockets.tcp.connect(createInfo.socketId,
                '192.168.4.1', 23, function(result){

                    GUI.log('Socket Connected');
                    self.connectionId = createInfo.socketId;
                    self.bitrate = 115200;
                    self.bytesReceived = 0;
                    self.bytesSent = 0;
                    self.failed = 0;
                    self.openRequested = false;
                    self.keepAlive();
                    self.useNaglesAlgo();
                    if(callback)callback(createInfo);

                });
        });
    },
    disconnect: function (callback) {
        var self = this;
        var result=1;
        if (self.connectionId) {
            self.emptyOutputBuffer();

            // remove listeners
            for (var i = (self.onReceive.listeners.length - 1); i >= 0; i--) {
                self.onReceive.removeListener(self.onReceive.listeners[i]);
            }

            for (var i = (self.onReceiveError.listeners.length - 1); i >= 0; i--) {
                self.onReceiveError.removeListener(self.onReceiveError.listeners[i]);
            }


            GUI.log("socket disconnected");
            chrome.sockets.tcp.disconnect(this.connectionId);


        } else {
            // connection wasn't opened, so we won't try to close anything
            // instead we will rise canceled flag which will prevent connect from continueing further after being canceled
            self.openCanceled = true;
        }
        if (callback) callback(result);

    },
    getDevices: function (callback) {
        chrome.serial.getDevices(function (devices_array) {
            var devices = [];
            devices_array.forEach(function (device) {
                devices.push(device.path);
            });

            callback(devices);
        });
    },
    getInfo: function (callback) {
        chrome.serial.getInfo(this.connectionId, callback);
    },
    getControlSignals: function (callback) {
        chrome.serial.getControlSignals(this.connectionId, callback);
    },
    setControlSignals: function (signals, callback) {
        chrome.serial.setControlSignals(this.connectionId, signals, callback);
    },
    send: function (data, callback) {
        var self = this;
        this.outputBuffer.push({'data': data, 'callback': callback});

        function send() {
            // store inside separate variables in case array gets destroyed
            var data = self.outputBuffer[0].data,
                callback = self.outputBuffer[0].callback;

            chrome.sockets.tcp.send(self.connectionId, data, function (sendInfo) {
                // track sent bytes for statistics
                self.bytesSent += sendInfo.bytesSent;


                //GUI.log("data sent to buffer "+data.toString());
                // fire callback
                if (callback) callback(sendInfo);

                // remove data for current transmission form the buffer
                self.outputBuffer.shift();

                // if there is any data in the queue fire send immediately, otherwise stop trasmitting
                if (self.outputBuffer.length) {
                    // keep the buffer withing reasonable limits
                    if (self.outputBuffer.length > 100) {
                        var counter = 0;

                        while (self.outputBuffer.length > 100) {
                            self.outputBuffer.pop();
                            counter++;
                        }

                        console.log('SERIAL: Send buffer overflowing, dropped: ' + counter + ' entries');
                    }

                    send();
                } else {
                    self.transmitting = false;
                    //GUI.log("not transmitting");
                }
            });
        }

        if (!this.transmitting) {
            this.transmitting = true;
            send();
        }
    },

    keepAlive: function()
    {

      chrome.sockets.tcp.setKeepAlive(this.connectionId,true,0,function(result){





      });

    },

    useNaglesAlgo:function () {

        chrome.sockets.tcp.setNoDelay(this.connectionId, true, function(result){





        });

    },


    onReceive: {
        listeners: [],

        addListener: function (function_reference) {
            chrome.sockets.tcp.onReceive.addListener(function_reference);
            this.listeners.push(function_reference);
        },
        removeListener: function (function_reference) {
            for (var i = (this.listeners.length - 1); i >= 0; i--) {
                if (this.listeners[i] == function_reference) {
                    chrome.sockets.tcp.onReceive.removeListener(function_reference);

                    this.listeners.splice(i, 1);
                    break;
                }
            }
        }
    },
    onReceiveError: {
        listeners: [],

        addListener: function (function_reference) {
            chrome.serial.onReceiveError.addListener(function_reference);
            this.listeners.push(function_reference);
        },
        removeListener: function (function_reference) {
            for (var i = (this.listeners.length - 1); i >= 0; i--) {
                if (this.listeners[i] == function_reference) {
                    chrome.serial.onReceiveError.removeListener(function_reference);

                    this.listeners.splice(i, 1);
                    break;
                }
            }
        }
    },
    emptyOutputBuffer: function () {
        this.outputBuffer = [];
        this.transmitting = false;
    }
};