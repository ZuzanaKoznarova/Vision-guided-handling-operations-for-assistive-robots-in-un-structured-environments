/**
 * @author      Zuzana Koznarova <zuzanazofka@gmail.com>
 * @version     1.0
 */
$fn=100;
bridgewidth=1;
footwidth=0.4;
pillarlength=3;
otherSizes=0.55;
pillarOtherSitzes=0.15;
rBall=3;
distanceBetweenFirstAndSecondFoot=0.3;
module pillarWithFoot(){
    cube([otherSizes,pillarOtherSitzes,pillarlength]);
    translate([0,-footwidth+pillarOtherSitzes,pillarlength-pillarOtherSitzes]){
        cube([otherSizes,footwidth,pillarOtherSitzes]);
        translate([0,-0.5*footwidth,-distanceBetweenFirstAndSecondFoot-pillarOtherSitzes]){
            cube([otherSizes,+1.5*footwidth,pillarOtherSitzes]);
        }
    }
}
module pillarOnRightPlace(scl=1){
        union(){
            scale(scl){
                rotate(a=180,v=[1,0,0]){
                    cube([otherSizes,bridgewidth,otherSizes]);
                    pillarWithFoot();
                    translate([otherSizes,bridgewidth,0]){
                        rotate(a=180, v=[0,0,1])
                        pillarWithFoot();
                    }
                }
            }
        }
}
//sphere(rBall);
module choosePart(part,scl=1){
    //to get from cm to mm, that are the units the scad
    scale(10){
        if(part=="upper"){
            translate([scl*otherSizes/2,-scl*bridgewidth/2,rBall-otherSizes]){
                difference(){    
                    sphere(rBall);
                          translate([0,0,-rBall]){
                    cube(rBall*2,center=true);
                    }
                }
            }
        }
            
        
        if(part=="bottom"){
            difference(){
            translate([scl*otherSizes/2,-scl*bridgewidth/2,rBall-otherSizes]){
                difference(){    
                    sphere(rBall);
                            translate([0,0,rBall]){
                    cube(rBall*2,center=true);
                    }
                }
            }
            pillarOnRightPlace(scl);
            }
        } 
            
        if(part=="bridge"){
            pillarOnRightPlace(scl);
        }
        if(part=="all"){
            translate([scl*otherSizes/2,-scl*bridgewidth/2,rBall-otherSizes]){    
                sphere(rBall);
            }
            pillarOnRightPlace(scl);
        }
    }
}

/* Please choose which part do zou want to draw, for easier 3D printing was the model devided in 3 parts:
upper hemisphere... choosePart("upper",1);
bottom hemisphere... choosePart("bottom",1.2); 
plug to the katana... choosePart("bridge",1);
whole model... choosePart("all",1);
Where the first Argument means the part of model and second describe the scale of the plug to the katana.
*/
//choosePart("upper",1);
//choosePart("bottom",1.2);
//choosePart("bridge",1.1);
choosePart("all",1);