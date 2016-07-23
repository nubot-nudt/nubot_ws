%% @file
% computes the cost function for the self localization for a given set of optimization parameters
%
% usage: J = costfcn(turtle)
%
% @author DJH Bruijnen <d.j.h.bruijnen@tue.nl>
% @date 2006
%
% @param p - optimization parameters [double array, 1x7]
% @param fieldmap - grid with distance to the most nearby line [uint8 array, size(fieldmap)]
% @param linepoints - linepoints in image coordinates [double array, mx2, 0<=m<=MAXLINEPOINTS]
% @return J = cost function value [double array, 1x1]

function J=costfcn(p,DistoMarkLine, fieldwidth, fieldlength, resolution, borderwidth, linepoints)

startx =-900-100;
endx   = 900+100;
starty =-600-100;
endy   = 600+100;
xlong  = endx-startx+1;
ylong  = endy-starty+1;
%p1,p2: center
%p3,p4: tan-parameters
%p5,p6,p7: orientation

%relative position in image  linepoints(:,1) x coordinate
X = linepoints(:,1) - p(1); 
Y = linepoints(:,2) - p(2);
phi = atan2(Y,X);
r = p(3)*tan(p(4)*sqrt(X.*X+Y.*Y))*100;


%local to global
sinphi = sin(phi);
cosphi = cos(phi);

x = p(5)+r.*(cosphi*cos(p(7))-sinphi*sin(p(7)));
y = p(6)+r.*(sinphi*cos(p(7))+cosphi*sin(p(7)));

J=0;
ref2=150.0*150.0;
d2=250.0*250.0;
c2=250.0*250.0;
nlines=size(linepoints,1);
for i =1:nlines
   x1=round(x(i));
   y1=round(y(i));
   if (abs(y1)<=endy&&abs(x1)<=endx)
	 templut = DistoMarkLine(round((y1-starty)*xlong+x1-startx+1)); 
   else 
     templut=500;
   end
   dist=templut;
   ef = c2+dist*dist;
   weights=(ref2+d2)/(d2+r(i)*r(i))*(1-d2/ef);
   J = J+weights;
end
%end
