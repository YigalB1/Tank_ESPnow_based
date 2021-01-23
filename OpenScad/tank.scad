$fn=30;



// this is the PCB, just for planning
//    color("green") translate([-20,14,0]) 
//        linear_extrude(height = 5, center = true, convexity = 10)
//            import("YigTank_PCB_2021-01-17_17-04-00.dxf", convexity=3);
          


tank();





module tank() {
    base_x = 155;
    base_y =  90;
    base_h = 1.5; // TBD - back to 1.5
    holes_d = 3;
    holes_h = 10;
    hole_1_dx = 45;
    hole_23_dx = 82.5;
    hole_23_dy = 29.5;
    dofen_h = 20;
    battery_holder_x = 20;
    battery_holder_d = 10;
    //battery_holder_y = 77;
    
    
    
    base();   
    battery_holder();
  
    
    module base() {
        pcb_x = 140;
        pcb_y = 75;

        color("green") screw_holders();
        
        difference() {
            cube(size=[base_x,base_y,base_h], center=true);
            holes(); // for the 3 screws from the Tank's base.
                     // and big 3 holes for wires to go down            
        } // of difference()
        // create 3 poles to hold the pcb
        base_poles();
        
        difference() {
            color("orange") dofen();
            // holes for the batteries
            color("red") rotate([0,90,0])
                translate([-dofen_h/2,base_y/4,-base_x/2])
                    cylinder(d=battery_holder_d,h=30,center=true);
            color("red") rotate([0,90,0])
                translate([-dofen_h/2,-base_y/4,-base_x/2])
                    cylinder(d=battery_holder_d,h=30,center=true);
        }
        
           module base_poles() {
               // create 3 poles to hold the pcb
               // have a larger base to create space between the PCB and the base
                pcb_pole_d = 2;
                pcb_pole_h = 15;
                hole_1_x = pcb_x/2-4.7;
                hole_1_y = pcb_y/2-5.3;
                hole_2_x = pcb_x/2-3.5;
                hole_2_y = -(pcb_y/2-3.2);
                hole_3_x = -(pcb_x/2-6.8);
                hole_3_y = (pcb_y/2-4.3);

               poles_base_d = 10;
               poles_base_h = 8;
               
               // create the base
               color("green") translate([hole_1_x,hole_1_y,poles_base_h/2])
                    cylinder(d=poles_base_d,h=poles_base_h,center=true);
               color("green") translate([hole_2_x,hole_2_y,poles_base_h/2])
                    cylinder(d=poles_base_d,h=poles_base_h,center=true);
               color("green") translate([hole_3_x,hole_3_y,poles_base_h/2])
                    cylinder(d=poles_base_d,h=poles_base_h,center=true);
                            
               //create the holes
               color("red") translate([hole_1_x,hole_1_y,pcb_pole_h/2])
                    cylinder(d=pcb_pole_d,h=pcb_pole_h,center=true);
               color("red") translate([hole_2_x,hole_2_y,pcb_pole_h/2])
                    cylinder(d=pcb_pole_d,h=pcb_pole_h,center=true);
               color("red") translate([hole_3_x,hole_3_y,pcb_pole_h/2])
                    cylinder(d=pcb_pole_d,h=pcb_pole_h,center=true);

        } // end of base_poles() module

            
            module screw_holders() {
                screw_d = 3;
                screw_h = 5;
                screw_xy = 5;
                
                x_offset = base_x/2-screw_xy/2-base_h/2;
                y_offset = base_y/2-screw_xy/2-base_h/2;
                
                // 4 corners
                translate([ x_offset, y_offset,0]) single_screw_holder();
                translate([-x_offset, y_offset,0]) single_screw_holder();
                translate([ x_offset,-y_offset,0]) single_screw_holder();
                translate([-x_offset,-y_offset,0]) single_screw_holder();
                
                // also 2 more on each side
                translate([ x_offset/3, y_offset,0]) single_screw_holder();
                translate([-x_offset/3, y_offset,0]) single_screw_holder();
                translate([ x_offset/3,-y_offset,0]) single_screw_holder();
                translate([-x_offset/3,-y_offset,0]) single_screw_holder();
                
                
                module single_screw_holder() {                    
                    difference(){
                        translate([0,0,dofen_h/2])
                            cube(size=[screw_xy,screw_xy,dofen_h],center=true);
                        translate([0,0,dofen_h-screw_h/2])
                            cylinder(d=screw_d,h=screw_h,center=true);
                    } // of difference
                } // of single_screw_holder() module
               

            }



    } // of base() module

    
 
    
    
    module dofen() {
        translate([0,base_y/2,dofen_h/2]) rotate([90,0,0])
            cube(size=[base_x,dofen_h,base_h], center=true);
        translate([0,-base_y/2,dofen_h/2]) rotate([90,0,0])
            cube(size=[base_x,dofen_h,base_h], center=true);
        translate([base_x/2,0,dofen_h/2]) rotate([90,0,90])
            cube(size=[base_y,dofen_h,base_h], center=true);
        translate([-base_x/2,0,dofen_h/2]) rotate([90,0,90])
            cube(size=[base_y,dofen_h,base_h], center=true);
    } // of dofen() module
    
    
    module battery_holder() {
        battery_9v_width = 27.5; // 26.4 in real - to create stable holder
        
        // last dofen
        translate([-(base_x/2+battery_holder_x),0,dofen_h/2])
            rotate([0,90,0])
                cube(size=[dofen_h,base_y,base_h], center=true);
        // base of battery extension
        translate([-(base_x/2+battery_holder_x-battery_holder_x/2),0,0])
            rotate([0,0,90])
                cube(size=[base_y,battery_holder_x,base_h], center=true);
        // dofen 1 of battery holder
        translate([-(base_x/2+battery_holder_x/2),-base_y/2,dofen_h/2])
            rotate([90,0,0])
                cube(size=[battery_holder_x,dofen_h,base_h], center=true);
        // dofen 2 of battery holder
        translate([-(base_x/2+battery_holder_x/2),base_y/2,dofen_h/2])
            rotate([90,0,0])
                cube(size=[battery_holder_x,dofen_h,base_h], center=true);
        // central dofen of battery holder
        translate([-(base_x/2+battery_holder_x/2),0,dofen_h/2])
            rotate([90,0,0])
                cube(size=[battery_holder_x,dofen_h,base_h], center=true);
                
                
        // left holder for battery
        translate([-(base_x/2+battery_holder_x/2),battery_9v_width,dofen_h/2])
            rotate([90,0,0])
                cube(size=[battery_holder_x,dofen_h,base_h], center=true);
        // left holder for battery
        translate([-(base_x/2+battery_holder_x/2),-battery_9v_width,dofen_h/2])
            rotate([90,0,0])
                cube(size=[battery_holder_x,dofen_h,base_h], center=true);                        
                
        
        
    } // of battery_holder() module
    
    
    
    module battery_holder_prev() {
        // last dofen
        translate([base_x/2+battery_holder_x,0,dofen_h/2]) rotate([0,90,0])
            cube(size=[dofen_h,base_y,base_h], center=true);
        // base of battery extension
        translate([base_x/2+battery_holder_x-battery_holder_x/2,0,0]) rotate([0,0,90])
            cube(size=[base_y,battery_holder_x,base_h], center=true);
        // dofen 1 of battery holder
        translate([base_x/2+battery_holder_x/2,-base_y/2,dofen_h/2]) rotate([90,0,0])
            cube(size=[battery_holder_x,dofen_h,base_h], center=true);
        // dofen 2 of battery holder
        translate([base_x/2+battery_holder_x/2,base_y/2,dofen_h/2]) rotate([90,0,0])
            cube(size=[battery_holder_x,dofen_h,base_h], center=true);
        // central dofen of battery holder
        translate([base_x/2+battery_holder_x/2,0,dofen_h/2]) rotate([90,0,0])
            cube(size=[battery_holder_x,dofen_h,base_h], center=true);
        
        
    } // of battery_holder() module
    
 
    
    
    
    module holes() {
        // the 3 small holes to connect to the base of the tank
        translate([base_x/2-hole_1_dx,0,0])
            cylinder(d=holes_d,h=holes_h,center=true);
        translate([base_x/2-hole_23_dx,hole_23_dy,0])
            cylinder(d=holes_d,h=holes_h,center=true);
        translate([base_x/2-hole_23_dx,-hole_23_dy,0])
            cylinder(d=holes_d,h=holes_h,center=true);
        
        
                  // 3 wide holes for wires to the motors
           color("red") rotate([0,0,0]) translate([base_x/2-20,0,0])
                    cylinder(d=battery_holder_d+10,h=30,center=true);
           color("red") rotate([0,0,0]) translate([-base_x/2+20,0,0])
                    cylinder(d=battery_holder_d+10,h=30,center=true);
           color("red") rotate([0,0,0]) translate([0,0,0])
                    cylinder(d=battery_holder_d+10,h=30,center=true);
        
        

        
    } // of holes() module
    
    
} // of tank() module

