// cvp - camera perspective (viewpoint)

// ECMAScript module

//npm install https://github.com/PeterTadich/elementary-rotations https://github.com/PeterTadich/homogeneous-transformations https://github.com/PeterTadich/matrix-computations

import * as hlao from 'matrix-computations';
import * as mcht from 'homogeneous-transformations';
import * as mcer from 'elementary-rotations';
//import * as hlao from '../matrix-computations/hlao.mjs';
//import * as mcht from '../homogeneous-transformations/mcht.mjs';
//import * as mcer from '../elementary-rotations/mcer.mjs';

function plotRobot(Li){
    var T0 = [];
    var n = Li.length;
    
    for(var i=0;i<n;i=i+1){
        var T = mcht.Aij(Li[i]); //transform from from {Ai} to {Ai-1}
        //console.log(T);
        if(i == 0) T0[i] = T;
        else T0[i] = hlao.matrix_multiplication(T0[i-1],T);
        //console.log(T0[i]);
        //plotFrame(T0[i]);
        plotFrame(camera(T0[i]));
    }
}

//Draw the partial robot skeleton (legs only).
function drawRobotUsingT0(T0,colour){
    for(var i=0;i<(T0.length-1);i=i+1){
        var Ts = camera(T0[i]); //start
        var Te = camera(T0[i+1]); //end
        drawSkeleton([Ts[0][3],Ts[1][3]],[Te[0][3],Te[1][3]],colour);
    }
    
    //colourGlobal = 'orange';
    //plotFrame(camera(T_BaseFrame)); //'T_BaseFrame' refer RMC.mjs
    //colourGlobal = 'Black';
    
    for(var i=0;i<T0.length;i=i+1){
        if(i==0){
            colourGlobal = 'yellow';
            plotFrame(camera(T0[0]));
        } else if(i==(T0.length-1)){
            colourGlobal = 'red';
            plotFrame(camera(T0[T0.length-1]));
        } else {
            colourGlobal = 'Black';
            plotFrame(camera(T0[i]));
        }
        colourGlobal = 'Black';
    }
    //colourGlobal = 'green';
    //plotFrame(camera(T0[0]));
    //colourGlobal = 'red';
    //plotFrame(camera(T0[T0.length-1]));
    //colourGlobal = 'Black';
    //console.log("Drawn z0 to z11.");
}

function plotFrame(T,txt,col){
    //example: plotFrame(Aij(Links[0]));
    var a = {};
    
    a.o = [[T[0][3]],[T[1][3]],[T[2][3]]]; //translation
    a.x = [[T[0][0]],[T[1][0]],[T[2][0]]]; //rotation
    a.y = [[T[0][1]],[T[1][1]],[T[2][1]]];
    a.z = [[T[0][2]],[T[1][2]],[T[2][2]]];
    //console.log(a.o);
    
    //example from page 22 (REF: Robotics Vision and Control): translation + rotation
    /*
    a.o = [[ 1.0   ],[2.0   ],[0.0]];
    a.x = [[ 0.8660],[0.5   ],[0.0]];
    a.y = [[-0.5   ],[0.8660],[0.0]];
    a.z = [[ 0.0   ],[0.0   ],[1.0]];
    */
    
    //pure translation
    /*
    a.o = [[1.0],[2.0],[0.0]];
    a.x = [[1.0],[0.0],[0.0]];
    a.y = [[0.0],[1.0],[0.0]];
    a.z = [[0.0],[0.0],[1.0]];
    */
    
    drawFrame(a,txt,col);
}

function worldFrame(){
    var w = {};
    w.o = [[0.0],[0.0],[0.0]];
    w.x = [[1.0],[0.0],[0.0]];
    w.y = [[0.0],[1.0],[0.0]];
    w.z = [[0.0],[0.0],[1.0]];
    
    drawFrame(w);
}

var T_Camera = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
];

//var cameraView = "defaultView"; //turn.js
//var cameraView = "sideView";
//var cameraView = "topView";
var cameraView = "frontView";
//var cameraView = "isoMetricView";
function camera(Tw){
    Tw = hlao.matrix_multiplication(T_Camera,Tw);
    
    /*
    //original for QUT (robotics weeksheet)
    var Ry = mcer.Ry_elementary(-1.0*Math.PI/2.0);
    var Ty = [
        [Ry[0][0], Ry[0][1], Ry[0][2], 0.0],
        [Ry[1][0], Ry[1][1], Ry[1][2], 0.0],
        [Ry[2][0], Ry[2][1], Ry[2][2], 0.0],
        [     0.0,      0.0,      0.0, 1.0]
    ];

    var Rx = mcer.Rx_elementary(-1.0*Math.PI/2.0);
    var Tx = [
        [Rx[0][0], Rx[0][1], Rx[0][2], 0.0],
        [Rx[1][0], Rx[1][1], Rx[1][2], 0.0],
        [Rx[2][0], Rx[2][1], Rx[2][2], 0.0],
        [     0.0,      0.0,      0.0, 1.0]
    ];
    
    var Ts = hlao.matrix_multiplication(hlao.matrix_multiplication(Ty,Tx),Tw);
    */
    
    switch(cameraView){
        case "defaultView":
            var Ts = Tw;
            break;
        
        case "sideView":
            //Tu: SIDE VIEW (moves to the right)
            var Ry = mcer.Ry_elementary(Math.PI/2.0);
            var Ty = [
                [Ry[0][0], Ry[0][1], Ry[0][2], 0.0],
                [Ry[1][0], Ry[1][1], Ry[1][2], 0.0],
                [Ry[2][0], Ry[2][1], Ry[2][2], 0.0],
                [     0.0,      0.0,      0.0, 1.0]
            ];
            
            var Rz = mcer.Rz_elementary(Math.PI/2.0);
            var Tz = [
                [Rz[0][0], Rz[0][1], Rz[0][2], 0.0],
                [Rz[1][0], Rz[1][1], Rz[1][2], 0.0],
                [Rz[2][0], Rz[2][1], Rz[2][2], 0.0],
                [     0.0,      0.0,      0.0, 1.0]
            ];
            
            var Ts = hlao.matrix_multiplication(hlao.matrix_multiplication(Ty,Tz),Tw);
            break;
            
        case "topView":
            //Tu: TOP VIEW (moves to the right)
            var Ry = mcer.Ry_elementary(Math.PI/2.0);
            var Ty = [
                [Ry[0][0], Ry[0][1], Ry[0][2], 0.0],
                [Ry[1][0], Ry[1][1], Ry[1][2], 0.0],
                [Ry[2][0], Ry[2][1], Ry[2][2], 0.0],
                [     0.0,      0.0,      0.0, 1.0]
            ];
            
            var Rz = mcer.Rz_elementary(Math.PI);
            var Tz = [
                [Rz[0][0], Rz[0][1], Rz[0][2], 0.0],
                [Rz[1][0], Rz[1][1], Rz[1][2], 0.0],
                [Rz[2][0], Rz[2][1], Rz[2][2], 0.0],
                [     0.0,      0.0,      0.0, 1.0]
            ];
            
            var Ts = hlao.matrix_multiplication(hlao.matrix_multiplication(Ty,Tz),Tw);
            break;
            
        case "frontView":
            //Tu: FRONT VIEW
            var Rz = mcer.Rz_elementary(Math.PI/2.0);
            var Tz = [
                [Rz[0][0], Rz[0][1], Rz[0][2], 0.0],
                [Rz[1][0], Rz[1][1], Rz[1][2], 0.0],
                [Rz[2][0], Rz[2][1], Rz[2][2], 0.0],
                [     0.0,      0.0,      0.0, 1.0]
            ];
            var Ts = hlao.matrix_multiplication(Tz,Tw);
            break;
            
        case "isoMetricView":
            //Tu: ISOMETRIC VIEW
            //
            //    ^y                           ^y0
            //    |                            |
            //    | {world}                    | {z0}
            //    /----> x     aligns with     /----> x0
            //   /                            /
            // |/_                          |/_
            
            var Rz1 = mcer.Rz_elementary(5.0*Math.PI/8.0);
            var Tz1 = [
                [Rz1[0][0], Rz1[0][1], Rz1[0][2], 0.0],
                [Rz1[1][0], Rz1[1][1], Rz1[1][2], 0.0],
                [Rz1[2][0], Rz1[2][1], Rz1[2][2], 0.0],
                [     0.0,      0.0,      0.0, 1.0]
            ];
            
            var Ry = mcer.Ry_elementary(-1.0*Math.PI/4.0);
            var Ty = [
                [Ry[0][0], Ry[0][1], Ry[0][2], 0.0],
                [Ry[1][0], Ry[1][1], Ry[1][2], 0.0],
                [Ry[2][0], Ry[2][1], Ry[2][2], 0.0],
                [     0.0,      0.0,      0.0, 1.0]
            ];
            
            var Rz2 = mcer.Rz_elementary(-1.0*Math.PI/8.0);
            var Tz2 = [
                [Rz2[0][0], Rz2[0][1], Rz2[0][2], 0.0],
                [Rz2[1][0], Rz2[1][1], Rz2[1][2], 0.0],
                [Rz2[2][0], Rz2[2][1], Rz2[2][2], 0.0],
                [     0.0,      0.0,      0.0, 1.0]
            ];
            
            var Ts = hlao.matrix_multiplication(hlao.matrix_multiplication(hlao.matrix_multiplication(Tz1,Ty),Tz2),Tw);
            break;
            
        default:
            console.log('WARNING: Camera view error: no view selected.');
            break;
    }
    
    return Ts;
}

//JavaScript colour REF: http://www.w3schools.com/tags/ref_colornames.asp
var colourGlobal = 'Black';
function setColour(col){
    colourGlobal = col;
}

var scaleFrame = 0.5;
var scaleSkeleton = 50;
var scaleOther = 7.5;
var tools;
var paper;
function drawFrame(frame,txt,col){
    var scale = scaleSkeleton;
    
    //origin
    //var o = hlao.matrix_multiplication_scalar(frame.o,scaleFrame*20); //setup for TUlip
    var o = hlao.matrix_multiplication_scalar(frame.o,scale*20); //setup for Tu
    //var o = hlao.matrix_multiplication_scalar(frame.o,scale*100); //setup for function 'turns()'. See turn.js
    //var o = hlao.matrix_multiplication_scalar(frame.o,scale*250); //setup for function '???()'. See tracking_camers.js
    //var o = hlao.matrix_multiplication_scalar(frame.o,scale*scaleOther); //setup for biomechanics
    //x unit vector
    var x = hlao.matrix_multiplication_scalar(frame.x,scale*scaleFrame);
    //y unit vector
    var y = hlao.matrix_multiplication_scalar(frame.y,scale*scaleFrame);
    //z unit vector
    var z = hlao.matrix_multiplication_scalar(frame.z,scale*scaleFrame);
    
    var xy = getCanvasWH();
    var cx = xy[0]/2.0;
    var cy = xy[1]/2.0;
    //console.log(cx,cy);
    
    //IMPORTANT: vertical flipping.
    //flip - canvas has +y vertically down
    o[1][0] = -1.0*o[1][0];
    x[1][0] = -1.0*x[1][0];
    y[1][0] = -1.0*y[1][0];
    z[1][0] = -1.0*z[1][0];
    
    //draw the points
    //   - addition of vector components: unit vector + origin translation + canvas centre
    tools.beginPath();
    tools.moveTo(o[0][0]+cx,o[1][0]+cy);
    tools.arc(o[0][0]+cx,o[1][0]+cy,2,0,2.0*Math.PI);
    
    tools.moveTo(x[0][0]+o[0][0]+cx,x[1][0]+o[1][0]+cy);
    tools.arc(x[0][0]+o[0][0]+cx,x[1][0]+o[1][0]+cy,2,0,2.0*Math.PI);
    
    tools.moveTo(y[0][0]+o[0][0]+cx,y[1][0]+o[1][0]+cy);
    tools.arc(y[0][0]+o[0][0]+cx,y[1][0]+o[1][0]+cy,2,0,2.0*Math.PI);
    
    tools.moveTo(z[0][0]+o[0][0]+cx,z[1][0]+o[1][0]+cy);
    tools.arc(z[0][0]+o[0][0]+cx,z[1][0]+o[1][0]+cy,2,0,2.0*Math.PI);
    tools.closePath();
    tools.fill();
    
    //draw the axes
    tools.strokeStyle = colourGlobal;
    //   - x axis
    tools.beginPath();
    tools.moveTo(o[0][0]+cx,o[1][0]+cy);
    tools.lineTo(x[0][0]+o[0][0]+cx,x[1][0]+o[1][0]+cy);
    tools.closePath();
    tools.stroke();
    //   - y axis
    tools.beginPath();
    tools.moveTo(o[0][0]+cx,o[1][0]+cy);
    tools.lineTo(y[0][0]+o[0][0]+cx,y[1][0]+o[1][0]+cy);
    tools.stroke();
    tools.closePath();
    //   - z axis
    tools.beginPath();
    tools.moveTo(o[0][0]+cx,o[1][0]+cy);
    tools.lineTo(z[0][0]+o[0][0]+cx,z[1][0]+o[1][0]+cy);
    tools.stroke();
    tools.closePath();
    
    //draw the labels
    tools.font = 'bold 20px Courier New';
    if(typeof col != 'undefined') tools.fillStyle = col;
    else tools.fillStyle = 'black';
    tools.fillText('o', o[0][0]+cx,o[1][0]+cy);
    //   - x axis
    tools.fillText('x', x[0][0]+o[0][0]+cx,x[1][0]+o[1][0]+cy);
    //   - y axis
    tools.fillText('y', y[0][0]+o[0][0]+cx,y[1][0]+o[1][0]+cy);
    //   - z axis
    if(cameraView != "defaultView") tools.fillText('z', z[0][0]+o[0][0]+cx,z[1][0]+o[1][0]+cy);
    //   - frame title
    var offset = scale*scaleFrame/2.0;
    if(typeof txt != 'undefined') tools.fillText(txt, o[0][0]+cx-offset,o[1][0]+cy+offset);
    
    //
    tools.strokeStyle = 'black';
    colourGlobal = 'Black';
    tools.fillStyle = 'black';
}

function drawSkeleton(pts,pte,colour){
    //pts, point start
    //pte, point end
    
    var scale = scaleSkeleton*20;
    
    var xy = getCanvasWH();
    var cx = xy[0]/2.0;
    var cy = xy[1]/2.0;
    
    tools.strokeStyle = colour;
    tools.beginPath();
    tools.moveTo(pts[0]*scale+cx,-1.0*pts[1]*scale+cy); //scale and flip on 'y'
    tools.lineTo(pte[0]*scale+cx,-1.0*pte[1]*scale+cy); //scale and flip on 'y'
    tools.stroke();
    tools.closePath();
    tools.strokeStyle = 'black';
}

function canvasSetup(){
    //fine adjust axix (Frame A) and major grid line
    var divWidth = (document.getElementById("robotCanvas").offsetWidth);
    var divHeight = (document.getElementById("robotCanvas").offsetHeight);
    //this aslo works
    //REF: https://www.w3schools.com/jsref/prop_win_innerheight.asp
    //var divWidth = window.innerWidth;
    //var divHeight = window.innerHeight;
    
    var str_Width = "";
    var str_Height = "";
    
    str_Width = divWidth + "px";
    str_Height = divHeight + "px";
    
    //canvas script
    //REF: http://www.w3schools.com/tags/ref_canvas.asp
    //REF: http://www.w3schools.com/html/html5_canvas.asp
    //REF: http://www.w3schools.com/tags/tag_canvas.asp
    //REF: http://www.w3schools.com/tags/canvas_lineto.asp
    
    //adjust canvas
    //style attribute
    //REF: https://www.w3schools.com/jsref/prop_style_height.asp
    document.getElementById("robotCanvas").style.width = str_Width;   // IMPORTANT: Once set can not override. Don't use.
    document.getElementById("robotCanvas").style.height = str_Height;
    //canvas attributes
    document.getElementById("robotCanvas").width = divWidth;
    document.getElementById("robotCanvas").height = divHeight;
    
    paper = document.getElementById("robotCanvas");
    tools = paper.getContext("2d");
    tools.lineWidth = 1;
}

function clearCanvas(){
    tools.clearRect(0, 0, paper.width, paper.height);
}

function getCanvasWH(){
    //ref: Eloquent JavaScript, page 240.
    //ref: Dynamic HTML The Definitive Reference, page 468.
    //ref: JavaScript: The Definitive Guide, page 393.
    //ref: DOM Enlightenment, page 66.
    var canvasHeight = document.getElementById("robotCanvas").offsetHeight;
    var canvasWidth  = document.getElementById("robotCanvas").offsetWidth;
    //console.log('canvas width: ' + canvasWidth + ', canvas height: ' + canvasHeight);
    return [canvasWidth, canvasHeight];
}

function canvasDetails(){
    var boxCoords = document.getElementById("robotCanvas").getBoundingClientRect(); //coordinates are given relative to 'window'
    var boxHeight = boxCoords.bottom - boxCoords.top;
    var boxWidth = boxCoords.right - boxCoords.left;
    console.log('canvas width: ' + boxWidth + ', canvas height: ' + boxHeight);
    //console.log(boxCoords.height, boxCoords.width); //This should work.
    var scrollDetails = getScrollOffsets();
    console.log('scroll pageXOffset: ' + scrollDetails.x + ', scroll pageYOffset: ' + scrollDetails.y);
    var ViewportSize = getViewportSize();
    console.log('viewport inner width: ' + ViewportSize.w + ', viewport inner height: ' + ViewportSize.h);
}

function getScrollOffsets(w){
    w = w||window;
    if(w.pageXOffset != null) return {x: w.pageXOffset, y: w.pageYOffset};
}

function getViewportSize(w){
    w = w||window;
    if(w.innerWidth != null) return {w: w.innerWidth, h: w.innerHeight};
}

export {
    plotRobot,
    drawRobotUsingT0,
    plotFrame,
    worldFrame,
    T_Camera,
    cameraView,
    camera,
    colourGlobal,
    setColour,
    scaleFrame,
    scaleSkeleton,
    scaleOther,
    tools,
    paper,
    drawFrame,
    drawSkeleton,
    canvasSetup,
    clearCanvas,
    getCanvasWH,
    canvasDetails,
    getScrollOffsets,
    getViewportSize
};