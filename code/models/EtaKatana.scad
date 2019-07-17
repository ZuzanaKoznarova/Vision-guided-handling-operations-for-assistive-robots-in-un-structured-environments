/**
 * @author      Zuzana Koznarova <zuzanazofka@gmail.com>
 * @version     1.1
 * Scroll down for setting the important parameters
 */
$fn=100;
vehicleWidthDown=53;
vehicleDiagonalSmaller1=12;
narrowingY=sqrt(vehicleDiagonalSmaller1*vehicleDiagonalSmaller1/2);
vehicleWidthMiddle=vehicleWidthDown-2*narrowingY;
narrowingYCabin=sqrt(8*8/2);
vehicleWidthUpper=vehicleWidthMiddle-2*narrowingYCabin;
sideForward=20;
cabinY=40;
pillowX=vehicleWidthDown;
pillowY=15;
pillowZ=4;
pillowX2=vehicleWidthDown;
pillowY2=10;
pillowZ2=4;
narrowingSide=12;
narrowingCabin=8;
middleWheelY=35;
widthWheel=4.5;
radiusWheel=18.5/2;
radiusOfSmallWheel=8/2;
widthOfSmallWheel=3.5;
wholeLength=pillowY+cabinY+pillowY2+2.5+narrowingY;
distanceOfSmallWheelFromBeginning=wholeLength-15;
upTheGround=9;
aluminiumProfilWidth=2.5;
aluminiumProfilLength1=100;



module beams(bigger, smaller, high) {
	polyhedron(points = [
		[-bigger/2, -smaller/2, 0],
        [bigger/2, -smaller/2, 0],
        [-bigger/2, smaller/2, 0],
        [bigger/2, smaller/2, 0],
        [0,  0,  high]
	], faces = [
		[4, 1, 0],
		[4, 3, 1],
		[4, 2, 3],
		[4, 0, 2],
		[0, 1, 2],
		[2, 1, 3]
	]);
}





module narrowing(x,y,z,narrowing){
    color([0.9,0.9,0.9])
    difference(){
        cube([x,y,z]);
        translate([0,0,-0.1]){
            rotate(a=45, v=[0,0,1]){
                cube(narrowing);
            }
        }
        translate([x,0,-0.1]){
            rotate(a=45, v=[0,0,1]){
                cube(narrowing);
            }
        
        }
    }
}

module wheel(radius,width){
    translate([-width,0,radius]){
        rotate(a=90, v=[0,1,0]){
            cylinder(r=radius,h=width);
        }
    }
}


module outerWorkingEnvelope(polomer){
    colorOfTheEnvelope=[0.2,0.5,0.5,0.2];
    %color(colorOfTheEnvelope)
    sphere(polomer);
    }

//katana
module katana(visibility){
l1=20; //length of first lenk
r1=9/2; //raius of first lenk
l2=20;
r2=3.5/2;
l3=14;
r3=3.5/2;
l4=23;
r4=6/2;
j1=9/2; //radius of first joint
jh1=14; //high of first joint
j2=7/2; 
jh2=13;
j3=7/2;
jh3=11;
g1a=8.5; //gripper length
g2a=2; //gripper width
g3a=0.6; //gripper depth
    
colorOfTheRobot=[0.2,0.5,0.5];
translate([0,0,0]){
    color(colorOfTheRobot)
    cylinder(r=r1,h=l1);
}
translate([0,0,l1]){
    color(colorOfTheRobot)
    cylinder(r=r2,h=l2);
    if(visibility=="envelope"){
        outerWorkingEnvelope(l2+l3+l4);
    }
    rotate(a=90, v=[1,0,0]){
        color(colorOfTheRobot)
        cylinder(r=j1,h=jh1,center=true);
    }
}
translate([0,0,l1+l2]){
    color(colorOfTheRobot)
    cylinder(r=r3,h=l3);
    rotate(a=90, v=[1,0,0]){
        color(colorOfTheRobot)
        cylinder(r=j2,h=jh2,center=true);
    }
}
translate([0,0,l1+l2+l3]){
    color(colorOfTheRobot)
    cylinder(r=r4,h=l4);
    rotate(a=90, v=[1,0,0]){
        color(colorOfTheRobot)
        cylinder(r=j3,h=jh3,center=true);
    }
}
translate([0,0,l1+l2+l3+l4]){
    rotate(a=-45, v=[0,1,0]){
        color(colorOfTheRobot)
        cube([g1a,g2a,g3a]);
    }
}


translate([0,0,l1+l2+l3+l4]){
    rotate(a=180+45, v=[0,1,0]){
        color(colorOfTheRobot)
        cube([g1a,g2a,g3a]);
    }
}


}

module vehicle(){
    //MPS 400
    translate([0,0,upTheGround]){
        //pillow vepredu
        color([0,0,0])
        cube([pillowX,pillowY,pillowZ]);
        translate([0,pillowY,0]){
            narrowing(pillowX,narrowingY,pillowZ,narrowingSide);
        }



        //Cabin
        color([0.9,0.9,0.9])
        translate([narrowingY,pillowY+narrowingY,0]){
            CabinZ=sideForward+pillowZ+narrowingYCabin;
            difference(){
                cube([vehicleWidthMiddle,cabinY,CabinZ]);
                color([0,0,1])
                translate([-narrowingCabin/2,-0.05,CabinZ]){
                    rotate(a=45, v=[0,1,0]){
                        cube([narrowingYCabin, cabinY+0.1, narrowingYCabin]);
                    }
                }
                translate([-narrowingCabin/2+vehicleWidthMiddle,-0.05,CabinZ]){
                    rotate(a=45, v=[0,1,0]){
                        cube([narrowingYCabin, cabinY+0.1, narrowingYCabin]);
                    }
                }
            }
        }


    //pillow vzadu
        translate([vehicleWidthDown,cabinY+pillowY+narrowingY+pillowY2+2.5,0]){
            rotate(a=180, v=[0,0,1]){
                color([0,0,0])
                cube([pillowX2,pillowY2,pillowZ2]);
                translate([0,pillowY2,0]){
                    narrowing(pillowX2,narrowingY,pillowZ2,narrowingSide);
                    }
            }
        }
    }

    //Wheel
    color([0.6,0.6,0.6])
    translate([narrowingY,middleWheelY,0]){
        wheel(radiusWheel, widthWheel);
    }
    color([0.6,0.6,0.6])
    translate([narrowingY+vehicleWidthMiddle+widthWheel,middleWheelY,0]){
        wheel(radiusWheel, widthWheel);
    }


    translate([vehicleWidthDown/2+widthOfSmallWheel/2,distanceOfSmallWheelFromBeginning,0]){
        translate([-widthOfSmallWheel/2,0,radiusOfSmallWheel+upTheGround/2]){
            rotate(a=45,v=[1,0,0]){
                color([0.9,0.9,0.9])
                cube([widthOfSmallWheel+0.2,upTheGround,upTheGround],center=true);
            }
        }
        color([1,0,0]);
        wheel(radiusOfSmallWheel, widthOfSmallWheel);
    }
}
module gallows(gallowHigh, gallowLength, visibility){
    //gallow
    translate([vehicleWidthDown/2-aluminiumProfilWidth/2,wholeLength-pillowY2-aluminiumProfilWidth,upTheGround+pillowZ]){
            cube([aluminiumProfilWidth,aluminiumProfilWidth,gallowHigh]);
            translate([0,0,gallowHigh-aluminiumProfilWidth]){
                rotate(a=90,v=[1,0,0]){
                    if(gallowLength<0){
                        translate([0,0,gallowLength]){
                            cube([aluminiumProfilWidth,aluminiumProfilWidth,abs(gallowLength)]);
                        }
                    }else{
                        cube([aluminiumProfilWidth,aluminiumProfilWidth,abs(gallowLength)]);
                    }
                }
            }

        //Kinect
        kinectwidth=4;
        kinectHeight=6.5;
        kinectLength=25;
        pedestalKinectWidth=2;
        pedestalKinectHeight=7;
        pedestalKinectLength=11;
        kinectAngle=45;


        distance=(gallowHigh+pedestalKinectHeight+kinectHeight/2)/cos(kinectAngle);
        angleOfViewHorizontal=70;
        angleOfViewVertical=60;
        SizeOfTheSideBigger=2*(tan(angleOfViewHorizontal/2)*distance);
        SizeOfTheSideSmaller=2*(tan(angleOfViewVertical/2)*distance);
        startOfTheBeamZ=gallowHigh+pedestalKinectWidth/2+pedestalKinectWidth/2+upTheGround+pillowZ+aluminiumProfilWidth;
        kinectVievHigh=startOfTheBeamZ/cos(kinectAngle);
        translate([0,-gallowLength+(gallowLength/abs(gallowLength))*(pedestalKinectHeight/2),gallowHigh+pedestalKinectWidth/2]){
            
            translate([0,-kinectHeight,pedestalKinectWidth/2]){
                rotate(a=kinectAngle,v=[1,0,0]){
                    translate([0,sqrt(2*kinectHeight/2*kinectHeight/2),0]){
                        color([0,0,0])
                        cube([kinectLength, kinectHeight, kinectwidth], center=true);
                        rotate(a=270, v=[1,0,0]){
                            high=kinectVievHigh+SizeOfTheSideSmaller/2;
                            translate([0,0,-high]){
                                if(visibility=="beams"){
                                    color([0.8,0.8,0,0.4])    
                                    beams(SizeOfTheSideBigger, SizeOfTheSideSmaller,high);
                                }
                            }
                        }
                    }
                }
            }
            //pedestal for Kinect
            color([0,0,0])
            cube([pedestalKinectLength, pedestalKinectHeight, pedestalKinectWidth], center=true);
        }
    }
}

module additionalJoint(highOfAdditionalJoint, visibility){
        //additional joint
    //bocnice
        aluminiumProfilDiagonall=sqrt((aluminiumProfilWidth*aluminiumProfilWidth)*2);
        aluminiumProfilLength3=pillowX-narrowingY+aluminiumProfilDiagonall;
        translate([0,pillowY+narrowingY/2]){
            translate([narrowingY/2,0,0]){
                rotate(a=45, v=[0,0,1]){
                    cube([aluminiumProfilWidth,aluminiumProfilWidth,aluminiumProfilLength1]);
                }
                translate([narrowingY+vehicleWidthMiddle,0,0]){
                    rotate(a=45, v=[0,0,1]){
                        cube([aluminiumProfilWidth,aluminiumProfilWidth,aluminiumProfilLength1]);
                    }
                }
                translate([-aluminiumProfilDiagonall/2,(aluminiumProfilDiagonall-aluminiumProfilWidth)/2,aluminiumProfilLength1+aluminiumProfilWidth]){
                    rotate(a=90, v=[0,1,0]){
                        cube([aluminiumProfilWidth,aluminiumProfilWidth,aluminiumProfilLength3]);
                    }
                }
            }
        }
    translate([0,0,upTheGround]){
        platformX=vehicleWidthMiddle;
        platformY=pillowY+narrowingY;
        platformZ=aluminiumProfilWidth;
        translate([0,0,highOfAdditionalJoint]){
            color([0.9,0.9,0.9])
            cube([pillowX,pillowY,pillowZ]);
            translate([0,pillowY,0]){
                translate([vehicleWidthDown/2,0,platformZ]){
                    katana(visibility);
                }
                narrowing(pillowX,narrowingY,pillowZ,narrowingSide);
            }
        }
    }
}

module all(highOfAdditionalJoint=0,gallowHigh=104, gallowLength=10, visibility="beams"){ 
    difference(){
        union(){
            vehicle();
            additionalJoint(highOfAdditionalJoint, visibility);
            gallows(gallowHigh,gallowLength, visibility);
        }
        sizeOfTheCube=1000;
        translate([0,0,-sizeOfTheCube/2]){
            cube(sizeOfTheCube, center=true);
        }
    }
}
/*
Please choose camera parameters:
        highOfAdditionalJoint... simulate the vertical additional translate join in different height,
        gallowHigh... height of the gallow where is camera,
        gallowLength... distance from vertical part of the gallow to the camera in y axes,
        visibleFeature... can be 'beams' for camera beams or 'envelope' robot outher working envelope.
        
Parameter highOfAdditionalJoint can help us with visualization for choosing fitting parameters for the upper and lower limit of additional joint.

Parameters gallowHigh and gallowLength can help to choose the fittingly parameters of the gallow for good Field of View (FoV). The yellow transparently light symbolize the FoV of the camera.
*/
highOfAdditionalJoint=10;
gallowHigh=104;
gallowLength=20;
visibility="beams";
all(highOfAdditionalJoint,gallowHigh,gallowLength,visibility);