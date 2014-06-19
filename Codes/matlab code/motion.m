%{
/**@mainpage processing image and issuing instructions to the bot
 @author Group 7: Garvit Juniwal 08005008
				   Ravi Bhoraskar 08005002
				   Kunal Shah 08005005
				   Namit Katariya 08005007
 


 Date: 7th April 2010
 
*********************************************************************************/


/********************************************************************************

   Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/
%}


%MATLAB Code for image sensing based bot motion.

%%%%%%%Close all pending connections to hardware. 
clear all;

%Check for pending connections to serial ports
if(~isempty(instrfind))
    fclose(instrfind);
end

%Check for pending connections to camera
if(~isempty(imaqfind))
    stop(imaqfind);
end;
%%%%%%%All pending connections closed

%%%%%%%Set up the Hardware

%Set up the serial port
Xbee=serial('COM3');
fopen(Xbee);

%Set up the camera
mode='YUY2_640X480';
vid=videoinput('winvideo',2,mode); 
set(vid, 'FramesPerTrigger', Inf);
set(vid, 'ReturnedColorspace', 'rgb');
preview(vid);
%%%%%%%Hardware booted

%Give time to adjust camera to correct position over arena
pause;

%%%%%%%Threshold adjustment

%Get a snapshot
start(vid);
RGB = getsnapshot(vid);
stop(vid);
imshow(RGB);

%Ask user for threshold details
dim1 = str2double(input('Enter number of columns','s')); %Number of Columns
dim2 = str2double(input('Enter number of rows','s'));    %Number of Rows

frl = str2double(input('Enter front red low','s')); %Front red Low threshold
frh = str2double(input('Enter front red high','s')); %Front red high threshold
fgl = str2double(input('Enter front green low','s')); %Front green Low threshold
fgh = str2double(input('Enter front green high','s')); %Front green high threshold
fbl = str2double(input('Enter front blue low','s')); %Front blue Low threshold
fbh = str2double(input('Enter front blue high','s')); %Front blue high threshold

brl = str2double(input('Enter back red low','s')); %back red Low threshold
brh = str2double(input('Enter back red high','s')); %back red high threshold
bgl = str2double(input('Enter back green low','s')); %back green Low threshold
bgh = str2double(input('Enter back green high','s')); %back green high threshold
bbl = str2double(input('Enter back blue low','s')); %back blue Low threshold
bbh = str2double(input('Enter back blue high','s')); %back blue high threshold


%Setting up Global Variables
GRID_DIM = [ dim1 , dim2 ];
CAMERA_DIM = [ 640 480 ];                           %Camera Mode. Fixed
NEXT_GRID = [ 1 3 ; 2 3; 2 2; 2 1; 3 1; 3 2; 3 3 ]; %Hardcoded path to follow
cur_loc = 1;                                        %Current Location
GRID_SIZE = [ CAMERA_DIM(1)/GRID_DIM(1) CAMERA_DIM(2)/GRID_DIM(2) ];
MINAREA=200;
FRONT_END = [ frl, frh ; fgl, fgh; fbl , fbh];
BACK_END = [ brl, brh; bgl, bgh; bbl, bbh];

pause

%Endless loop
while(1)
    
    %get the snapshot
    start(vid);
    RGB = getsnapshot(vid);
    stop(vid);
    dim=size(RGB);
   
    locfront = zeros(dim(1), dim(2));%To store the binary image for the front portion
    locback  = zeros(dim(1), dim(2));%To store the binary image for the back portion
   
    %Mark the front region and back region of bot using threshold values in
    %the defn
   
    for i=1:(dim(1)-1)
        for j=1:(dim(2)-1)
            ff = 1;
            fb = 1;
            for k=1:3
                if not(RGB(i,j,k)<=FRONT_END(k,2) && RGB(i,j,k)>=FRONT_END(k,1))%Check if pixel does not belong to the front end
                    ff=0;
                end
                if not(RGB(i,j,k)<=BACK_END(k,2) && RGB(i,j,k)>=BACK_END(k,1))%Check if the pixel does not belong to the back end
                   fb=0;
                end
            end
           
            if ff==1
                locfront(i,j)=255;
            end
            if fb==1
                locback(i,j)=255;
            end
        end
    end
    imshow(RGB)
    
    %Parse the front end
    diff_fe = bwareaopen(locfront,MINAREA);   %Remove all areas with area < MINAREA
    fe_label= bwlabel(diff_fe,8);             %Label all remaining areas
    imshow(diff_fe);
    
    stats_fe = regionprops(fe_label, 'basic');%Find the region properties
    hold on;
    for object=1:length(stats_fe)             %For each of the remaining areas
        bb=stats_fe(object).BoundingBox;      %Find the bounding box 
       
        area=stats_fe(object).Area;           
        if(area>MINAREA)                      
           x=bb(1);
           y=bb(2);
           w=bb(3);
           h=bb(4);
           front_center_x = bb(1)+(bb(3)/2);  %Find the coordinates of the center
           front_center_y = bb(2)+(bb(4)/2);  %Of the bounding box
           rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
        end
    end
    
    %Parse the back end
    diff_be = bwareaopen(locback,MINAREA);    %Remove all areas with area < MINAREA   
    be_label= bwlabel(diff_be,8);             %Label all remaining areas           
    imshow(diff_be);
    
    stats_be = regionprops(be_label, 'basic');%Find the region properties
    
    for object=1:length(stats_be)             %For each of the remaining areas
        bb=stats_be(object).BoundingBox;      %Find the bounding box 
       
        area=stats_be(object).Area;
        if(area>MINAREA)
           x=bb(1);
           y=bb(2);
           w=bb(3);
           h=bb(4);
           back_center_x = bb(1)+(bb(3)/2);  %Find the coordinates of the center
           back_center_y = bb(2)+(bb(4)/2);  %Of the bounding box
           rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
        end
    end
    hold off;
    
    %Centers found.
    
    
    %Find grid in which the back end is located
    cur_grid = [ ceil(back_center_x/GRID_SIZE(1)), ceil(back_center_y/GRID_SIZE(2)) ];
      
    %If we have already reached where we wanted to, update that information
    if(  cur_grid == NEXT_GRID(cur_loc,:) )
       cur_loc = cur_loc+1;
       if(cur_loc == length(NEXT_GRID) )
           'Traversal Completed';
           break;
       end
       
    end
    
    %Calculate the centers of the grid location where we are, and where 
    %we want to go next
    next_center_x = (NEXT_GRID(cur_loc,1)*GRID_SIZE(1)) - (GRID_SIZE(1)/2);
    next_center_y = (NEXT_GRID(cur_loc,2)*GRID_SIZE(2)) - (GRID_SIZE(2)/2);
    center_x = (cur_grid(1)*GRID_SIZE(1)) - (GRID_SIZE(1)/2);
    center_y = (cur_grid(2)*GRID_SIZE(2)) - (GRID_SIZE(2)/2);
    
    %Now we calculate two parameters to judge the next direction of motion
    nextLocation = [next_center_x next_center_y 0];
    currentLocation = [center_x center_y 0];
   
    dir1 = [front_center_x front_center_y 0] - [back_center_x back_center_y 0];
    dir2 = nextLocation - currentLocation ;
    temp = cross( dir1 , dir2 )/(norm(dir1)*norm(dir2));
    anglesin = -temp(3);
    anglecos = dot(dir1, dir2)/(norm(dir1)*norm(dir2));
    
    
    if anglesin >0 && anglecos >0
       angle = acosd(anglecos);
    end
    if anglesin <0  && anglecos>0
        angle=-1*acosd(anglecos);       
    end
    if anglesin >0 && anglecos <0
        angle = asind(anglesin);
    end
    if anglesin<0 && anglecos<0
        angle = -1*asind(abs(anglesin));
    end
   
    P = [ (front_center_x + back_center_x)/2 (front_center_y + back_center_y)/2 0 ];
    vertdist = norm( cross( currentLocation - nextLocation , P - nextLocation ) / norm( currentLocation - nextLocation ));
%     
%     Now, depending upon the position and orientation of the robot,
%     we shall identify the 'case' which we are in. The cases, along with
%     the action we shall take on them are as follows:
%     0. Robot Oriented in the direction we want to go, but in correct grid
%         Go straight at full speed
%     1. Robot slightly misoriented towards the right, but in correct grid
%         Soft turn left
%     2. Robot deeply misoriented towards the right, but in correct grid
%         Hard turn left
%     3. Robot slightly misoriented towards the left, but in correct grid
%         Soft turn right
%     4. Robot deeply misoriented towards the left, but in correct grid
%         Hard turn right
%     5. Robot is not on the close to the center but moving in the right direction
%         If the bot is facing right do soft right.
%         If the bot is facing left do soft left
%     6. Robot is not on the close to the center and moving in the wrong direction
%         Move hard left if facing right and hard right if facing left
%    
    
    if( vertdist < GRID_SIZE(2)/5 )
        if(angle < 10 && angle > -5)
            'case 0'
            fwrite(Xbee, 0);
        end
        if( angle < 15 && angle >= 5)
            fwrite(Xbee, 1);
            'case 1'
        end
        if(angle >=15 )
            'case 2'
            fwrite(Xbee, 2);
        end
        if(angle<-5 && angle>-15)
            'case 3'
            fwrite(Xbee, 3);
        end
        if(angle <-15)
            'case 4'
            fwrite(Xbee, 4);
        end
    else
        new_P = P+ (10* (dir1/norm(dir1)));
        new_vert_dist = norm( cross( currentLocation - nextLocation , new_P - nextLocation ) ) / norm( currentLocation - nextLocation );
        if( new_vert_dist < vertdist )
            if( angle < 0 ) 
                'case 5a'
                fwrite(Xbee, 1);
            else
                'case 5b'
                fwrite(Xbee, 3);
            end
        else
            
            if(angle>0)
                'case 6a'
                fwrite(Xbee, 6);
            else
                'case 6b'
                fwrite(Xbee, 7);
            end
        end
    end
    pause(1); %Give bot time to process motion.
end
