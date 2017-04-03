var arDrone = require('ar-drone');
var client = arDrone.createClient();
var Project;
var serialport = require('node-serialport')
var sp = new serialport.SerialPort("/dev/ttyO3", {
  parser: serialport.parsers.readline("\n"),
  baud: 9600
})

//DRONE NAVIGATION VARIABLES NEEDED
var ROBOT = new Array(0, 0, 0);  //drone starts at x/y/z origin point
var PITCH = 0;
var ROLL = 0;
var YAW = 0;
var FRONT = 0;
var LEFT = 0;
var RIGHT = 0;
var TOP = 0;
var BOTTOM = 0;

//DRONE OPERATION TIME
CYCLE = 0   //don't change this
TOTAL_OPERATION_CYCLES = 10000  //how many times the drone should operate.  Edit as needed.


//sleep code from http://stackoverflow.com/questions/951021/what-is-the-javascript-version-of-sleep
function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

function add_forces(a, b){
    //This function adds to force vectors together and returns the result
    if (a.length != b.length){
        throw "Force vectors differ in length";
    }
    var c = new Array(a.length);
    for (i = 0; i < a.length; i++){
        c[i] = a[i] + b[i];
    }
    return c
}

function wrap_angle(angle){
    while(angle >= Math.PI){
        angle = angle - 2*Math.PI;
    }
    while(angle <= -Math.PI){
        angle = angle + 2*Math.PI;
    }
    return angle;
}

function get_obstacle_force(distance, angle, axis){
    var force_from_obstacles = new Array(0, 0, 0);

    var m = get_pf_magnitude_linear(distance);  //need to adjust for angle?

    //flip angle since force should go opposite obstacle direction
    if(cur_angle >= 0) {
        d = wrap_angle(angle + 180) //direction
    }
    else{
        d = wrap_angle(angle - 180) //direction
    }

    //axis tells whether the force should affect the x, y, or z axis
    if(axis === "x"){
        new_x = force_from_obstacles[0] + (m * Math.cos(d));
        new_y = force_from_obstacles[1] + (m * Math.sin(d));
        new_z = force_from_obstacles[2] + (m * Math.tan(d));
    }
    if(axis === "y"){
        new_x = force_from_obstacles[0] + (m * Math.tan(d));
        new_y = force_from_obstacles[1] + (m * Math.cos(d));
        new_z = force_from_obstacles[2] + (m * Math.sin(d));
    }
    if(axis === "z"){
        new_x = force_from_obstacles[0] + (m * Math.sin(d));
        new_y = force_from_obstacles[1] + (m * Math.tan(d));
        new_z = force_from_obstacles[2] + (m * Math.cos(d));
    }

    force_from_obstacles[0] = new_x;
    force_from_obstacles[1] = new_y;
    force_from_obstacles[2] = new_z;

    return force_from_obstacles;
}

function goal_force(){  //TODO

    //return uniform force, no goal point right now
    var strength = 0.25;  //magnitude of uniform force
    return new Array(0, 0, 0);
}

function obstacle_force(){

    front_force = 0 - get_obstacle_force(FRONT, YAW, "y");
    right_force = 0 - get_obstacle_force(RIGHT, ROLL, "x");
    left_force = 0 - get_obstacle_force(LEFT, -ROLL, "x");
    top_force = 0 - get_obstacle_force(TOP, PITCH, "z");
    bottom_force = 0 - get_obstacle_force(BOTTOM, -PITCH, "z");

    //add forces
    total_force = new Array(0, 0, 0);
    total_force[0] = front_force[0] +   //x components
            right_force[0] +
            left_force[0] +
            top_force[0] +
            bottom_force[0];

    total_force[1] = front_force[1] +  //y components
            right_force[1] +
            left_force[1] +
            top_force[1] +
            bottom_force[1];

    total_force[2] = front_force[2] +  //z components
            right_force[2] +
            left_force[2] +
            top_force[2] +
            bottom_force[2];

    return total_force;
}

function drive_from_force(total_force){ //TODO

}

function get_pf_magnitude_linear(distance){
    //How close to the obstacle do we have to be to begin feeling repulsive force
    var distance_threshold = 1.0;

    //The maximum strength of the repulsive force
    var max_strength = 20.0;

    //   1. Compute the magnitude of the force for the given distance and return it
    if (distance < distance_threshold){
        var strength = max_strength * (distance_threshold - distance);
        return strength;
    }

    return 0;

}

function update_goal_status(){
    //should make drone operation last for only a temporary time period
    if(CYCLE < TOTAL_OPERATION_CYCLES){
        CYCLE++;
        return 1;
    }
    else{
        return 0;
    }
}

function potential(){  //TODO:  NEED TO DRIVE AND HAVE VARIABLE UPDATES

    var GoalNotMet = 1;
    var rate = 5;  //set sleep rate time in ms

    while(GoalNotMet){
        //TODO:  UPDATE NAVIGATION VARIABLES

        var g_force = goal_force();
        var o_force = obstacle_force();
        var total_force = add_forces(g_force, o_force);
        //TODO:  Must figure out how to add together multiple driving directions
        //var twist = drive_from_force(total_force);

        //TODO:  publish "twist" command - must figure out JS equivalent

        //sleep this function for a short time.
        await sleep(rate);

        //make sure drone not flying while new directions are found
        //var twist = Twist();
        //publish(twist);

        //Check if goal met
        GoalNotMet = update_goal_status();

        //then repeat
    }
}

function main(){ //TODO
    client.after(1000, function() {  //start
    this.takeoff();
    })

    potential();  //navigate autonomously

    client.after(1000, function() {  //finish
        this.land();
    })
}





//need to figure out how to incorporate this to get data how we would like
sp.on('data', function(chunk) {
  Project = chunk.toString();
  console.log("%s",Project);
  if(Project== 'T'){
   client.stop();
   client.land();
   client.animateLeds('blinkRed', 5 ,2);
 console.log("Drone Landed\n");
  }
  if(Project == 'F'){
  client.front(0.08);
  }
  if(Project== 'L'){
  client.left(0.06);
  }
  if(Project== 'R'){
  client.right(0.06);
  }
  if(Project== 'Q'){
  client.left(0.06);
  }
  if(Project== 'P'){
  client.right(0.06);
  }
  if(Project== 'S'){
  client.stop();
  }
});


