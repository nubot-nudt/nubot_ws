clear 
clc
DistoMarkLine1=load('errortable.txt');
startx =-900-100;
endx   = 900+100;
starty =-600-100;
endy   = 600+100;
xlong  =endx-startx+1;
ylong  =endy-starty+1;
idx=1;
for i=starty:0
    for j=startx:0
        DistoMarkLine((i-starty)*xlong+j-startx+1)= DistoMarkLine1(idx);
        DistoMarkLine((ylong-1-(i-starty))*xlong+j-startx+1)=DistoMarkLine1(idx);
	    DistoMarkLine((i-starty)*xlong+xlong-1-(j-startx)+1)=DistoMarkLine1(idx);
	    DistoMarkLine((ylong-1-(i-starty))*xlong+xlong-1-(j-startx)+1)=DistoMarkLine1(idx);
        idx=idx+1;
    end
end

fieldwidth =  1200 ;
fieldlength = 1800 ;
resolution =  1;
borderwidth = 12;
fmx = fieldwidth/2+borderwidth;
fmy = fieldlength/2+borderwidth;


%%  x y coordinates
linepoints=load('whites.txt'); 

%% get middle position
xmlDoc=xmlread('mirror_calib.xml');
IDArray = xmlDoc.getElementsByTagName('center_coloum');            % the y coordinate
p(1) =  str2num( char(IDArray.item(0).getFirstChild.getData) )  ;
IDArray = xmlDoc.getElementsByTagName('center_row');     
p(2) =  str2num ( char(IDArray.item(0).getFirstChild.getData) ) ;  % the x coordinate
IDArray = xmlDoc.getElementsByTagName('para_a');     
p(3) =  str2num ( char(IDArray.item(0).getFirstChild.getData) ) ;  % the para_a coordinate
 IDArray = xmlDoc.getElementsByTagName('para_b');     
p(4) =  str2num ( char(IDArray.item(0).getFirstChild.getData) ) ;  % the para_b coordinate


figure(1);
Image1=imread('pic_save0.bmp');
imshow(Image1)
hold on;
plot(linepoints(:,1),linepoints(:,2),'*y')


%% plot results
%relative position in image  linepoints(:,1) x coordinate
X = linepoints(:,1) - p(1); 
Y = linepoints(:,2) - p(2);
phi = atan2(Y,X);
r = p(3)*tan(p(4)*sqrt(X.*X+Y.*Y))*100;

%local to global
sinphi = sin(phi);
cosphi = cos(phi);

x = r.*(cosphi*cos(0)-sinphi*sin(0));
y = r.*(sinphi*cos(0)+cosphi*sin(0));
figure(2)
plot(x,-y,'r*')
hold on;
grid minor


