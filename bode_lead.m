% Author: Xin Lyu
% Output: K,T, alpha
% Input: np,dp,wc,PM
% Compute the lead compensation of the plant np/dp to have
% Phase Margin at crossover frequency (in radians/sec)
% e.g. [K,T,alpha] = bode_lead([1],[1 1 0],10,60)
function [K,T,alpha] = bode_lead(np,dp,wc,PM)
% first compute the plant response at wc
[magc,phc] = bode(np,dp,wc);

% compute the needed phase lead at wc in radians 
phir =(-180+PM-phc)*pi/180;

% Set boundry between +- 90 degrees
if abs(phir)>pi/2;
    fprintf('A simple phase lead/lag cannot change the phases by more\n');
    fprintf('than+/- 90 degrees.\n'); 
    error('Aborting.')
end

% compute alpha, T, and K for compensator

alpha = (1-sin(phir))/(1+sin(phir));
T = 1/(wc*sqrt(alpha));
K =sqrt(alpha)/magc;
% compute the new open-loop system by convolving the plant polynomials
% with the compensator polynomials
nol = conv(np,K*[T 1]);
dol = conv(dp,[alpha*T 1]);

% check the solution by plotting the bode plot for the new open loop
% polynomials. 

w = logspace(-2,1)*wc;
w(34)=wc; clf;
[mag1,ph1]=bode(np,dp,w); [mag2,ph2]=bode(nol,dol,w);
subplot(211);
semilogx(w/wc,20*log10(mag1),'--'); hold on;
semilogx(w/wc,20*log10(mag2)); grid; plot(w/wc,0*w+1,'g-');
ylabel('Magnitude (dB)');
title('Phase-lead design (uncompensated=dashed; compensated=solid)');
subplot(212);
semilogx(w/wc,ph1,'--'); hold on; semilogx(w/wc,ph2); grid;
plot(w/wc,0*w-180+PM,'g-');
ylabel('Phase'); xlabel('Frequency/wc (i.e., "1"=wc)');

end

