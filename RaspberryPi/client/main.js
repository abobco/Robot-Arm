let  player = 0;

function get_appropriate_ws_url(extra_url)
{
    var pcol;
    var u = document.URL;

    /*
    * We open the websocket encrypted if this page came on an
    * https:// url itself, otherwise unencrypted
    */

    if (u.substring(0, 5) === "https") {
        pcol = "wss://";
        u = u.substr(8);
    } else {
        pcol = "ws://";
        if (u.substring(0, 4) === "http")
            u = u.substr(7);
    }

    u = u.split("/");

    /* + "/xxx" bit is for IE10 workaround */
    return pcol + u[0] + "/" + extra_url;
}

function new_ws(urlpath, protocol) {
    let ws = new WebSocket(urlpath, protocol);
    ws.binaryType = "arraybuffer"
    return ws;
}  

let debug = false;
document.addEventListener("DOMContentLoaded", () => {    
    var ws = new_ws(get_appropriate_ws_url(""), "lws-minimal");
    console.log(ws.binaryType);
    try {
        ws.onopen = ( evt ) => { };

        ws.onmessage = function got_packet(msg) {           
            if ( !player ) { // 1st message we received

                let data_view = new Int32Array(msg.data);
                player = data_view[0]; // set player #
                console.log(player);
                
                const pb = document.getElementById('permission-button');
                // add a callback function
                pb.addEventListener('click', () => {
                    // feature detect
                    if (typeof DeviceMotionEvent.requestPermission === 'function') {
                        // This opens a popup asking for permission & returns a promise
                        DeviceMotionEvent.requestPermission().then( permissionState => { // attach a callback to the promise    
                            if (permissionState === 'granted') 
                                motion_setup(ws);   // start sending gyro data
                        }).catch(console.error);
                    } else {
                        motion_setup(ws);
                    }
                });
                
            }
        };

        ws.onclose = function() {};
        
        // ---------------- button callbacks -----------------------------------

        function create_slider(ws, slider, output, servo_id ) {
            // let slider = document.getElementById(slider_dom_id);
            // let output = document.getElementById(val_dom_id);
            output.innerHTML = servo_id.toString() + ": " + slider.value;
        
            slider.oninput = function() {
                output.innerHTML = servo_id.toString() + ": " + this.value;
                let buffer = new ArrayBuffer(8);
                let typed_array = new Int32Array(buffer);
                
                // quit message values
                typed_array[0] = this.value;
                typed_array[1] = servo_id;
        
                // write to the socket
                ws.send(buffer); 
            }
        }

        function add_attribute(ele, type, val) {
            let att = document.createAttribute(type);
            att.value = val;
            ele.setAttribute(type, val);
        }
        
        let sliders = document.getElementById("sliders");
        for ( let i =0; i < 6; i++ ) {
            let s = document.createElement("input");
            add_attribute(s, "type", "range");
            add_attribute(s, "min", 500);
            add_attribute(s, "max", 2500);
            if ( i == 2)
                add_attribute(s, "value", 500);
            else if ( i == 3)
                add_attribute(s, "value", 2500);
            else if ( i == 4)
                add_attribute(s, "value", 2500);
            else
                add_attribute(s, "value", 1500);
            add_attribute(s, "class", "slider");
            add_attribute(s, "id", i.toString() );

            let v = document.createElement("p");
            v.innerHTML = "Val";

            let sp = document.createElement("span");
            add_attribute(sp, "id", "v" + i.toString());

            v.appendChild(sp);
            // s.appendChild(v);
            
            sliders.appendChild(v);
            sliders.appendChild(s);

            create_slider(ws, s, v, i+1);
        }

        // ---------------------------------------------------------------------
    } catch(exception) {
        alert("<p>Error " + exception);  
    }
    
}, false);
