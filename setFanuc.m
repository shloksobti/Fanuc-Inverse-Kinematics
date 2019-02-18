% Lab 2
% setFanuc.m

function setFanuc( angles, fanuc )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 2
% Solutions by Craig McDonald
%    DESCRIPTION - Update the position of the FANUC after calling drawFanuc()
% 
%    This function can be used as is once fanucFK() and drawFanuc() have been
%    completed.
% 
% 

[~,fanuc_T] = fanucFK(angles,fanuc);
set(fanuc.handles(1),'Matrix',fanuc_T{1});
set(fanuc.handles(2),'Matrix',fanuc_T{2});
set(fanuc.handles(3),'Matrix',fanuc_T{3});
set(fanuc.handles(4),'Matrix',fanuc_T{4});
set(fanuc.handles(5),'Matrix',fanuc_T{5});
set(fanuc.handles(6),'Matrix',fanuc_T{6});
visibility = {'off','on'};
set(fanuc.handles(7),'Visible',visibility{double(fanuc.brush==1)+1});
set(fanuc.handles(8),'Visible',visibility{double(fanuc.brush==2)+1});
set(fanuc.handles(9),'Visible',visibility{double(fanuc.brush==3)+1});
set(fanuc.handles(10),'Visible',visibility{double(fanuc.brush==4)+1});
drawnow;

end
