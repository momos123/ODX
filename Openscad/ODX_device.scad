
 difference(){
 union() {
   translate([-21.0,-21,0]) 
   color([0,0.3,0.3, 1])
  cube([56.5,42,44]);//Main Box

 }
 union() {
   translate([0,0,-8.8]) 
   cylinder(d=29.5, h=45, $fn=100);  //cuvette place
     
   translate([15,0,21.5]) 
   cube([18,8,16], center=true);//optical sensor window
     
   rotate([180,90,0]) 
   translate([-22,0,12]) //LED1
   cylinder(h=19,d=5.2,$fn=100,center=true);
     
     
   translate([-20.0,0,41]) 
   cube([5.1,10.5,6.1], center=true);//Switch
     
   translate([-20.0,0,33]) 
   cube([3,4,20], center=true);//LEd1 Window
   

   translate([14.0,0,43]) 
   cube([22,8,6], center=true);//wrires  Window

   rotate([90,90,0]) 
   translate([-21,27.5,18]) 
   cube ([41.0,12.5,55], center=true);  //for fitness tracker  
   
  
   rotate([-90,90,0]) 
   translate([-6,-28,12]) 
   cylinder(h=19,d=6,$fn=100,center=true); //To remove the fitness tracker
   
   rotate([-90,90,0]) 
   translate([-22,-28,12]) 
   cylinder(h=19,d=10,$fn=100,center=true); //To remove the fitness tracker
   
   rotate([-90,90,0]) 
   translate([-36,-28,12]) 
   cylinder(h=19,d=6,$fn=100,center=true); //To remove the fitness tracker
   
   translate([-18.0,-18,37.5]) 
   color([1,0.3,0.3, 0.5])
   cube([36,36,20]);//Electronics Box
  
   rotate([-90,90,0]) 
   translate([-25,-34,0]) 
   cube([29,5,16], center=true);//oled screen
   
 }
 }



   