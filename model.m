clear;
clc;

syms x xd xdd g T Tp N R Iw mw mp Ip thetadd thetad theta P M L PM LM NM LM IM phidd phid phi l;
% 
% fu1=N-NM==mp*diff(diff(x+L*sin(theta),theta),theta);
% fu2=P-PM-mp*g==mp*diff(diff(L*cos(theta),theta),theta);
% fu3=NM==M*diff(diff(x+(L+LM)*sin(theta),theta)-l*sin(phi),phi);
% fu4=PM-M*g==M*diff(diff((L+LM)*cos(theta,theta)+l*cos(phi),phi));


fu1=N-NM==mp*(xdd+L*(thetadd*cos(theta)-thetad*thetad*sin(theta)));
fu2=P-PM-mp*g==mp*L*(-thetadd*sin(theta)-thetad*thetad*cos(theta));
fu3=NM==M*(xdd+(L+LM)*(thetadd*cos(theta)-thetad*thetad*sin(theta))-l*(phidd*cos(phi)-phid*phid*sin(phi)));
fu4=PM-M*g==M*((L+LM)*(-thetadd*sin(theta)-thetad*thetad*cos(theta))+l*(-phidd*sin(phi)-phid*phid*cos(phi)));
%
[N,NM,P,PM]=solve(fu1,fu2,fu3,fu4,N,NM,P,PM);
f1=xdd==(T-N*R)/(Iw/R+mw*R);
f2=Ip*thetadd==(P*L+PM*LM)*sin(theta)-(N*L+NM*LM)*cos(theta)-T+Tp;
f3=IM*phidd==Tp+NM*l*cos(phi)+PM*l*sin(phi);
[xdd,thetadd,phidd]=solve(f1,f2,f3,xdd,thetadd,phidd);
%
func=[thetad,thetadd,xd,xdd,phid,phidd];
lin_model=jacobian(func,[theta,thetad,x,xd,phi,phid]);
temp=subs(lin_model,[theta,thetad,xd,phi,phid],zeros(1,5));
final_lin_model=simplify(temp)


            