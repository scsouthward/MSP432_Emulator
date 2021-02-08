function vout = plant_emulator(vin,fs,init)

persistent xk DTsys 
global VMAXP VMINP

%  --check for invalid inputs
if (nargin == 0)
    %  --assume that the user is probing for validity
    vout = 'valid';
    return
elseif (nargin ~= 3)
    error('Invalid call to plant emulator')
end

%  --initialize the local parameters
if init
    %  --initialize 2nd order system parameters
    wn = 2*pi*10;   % frequency (rad/s)
    zeta = 0.3;     % damping ratio
    
    %  --CT equations of motion
    num = 10*wn^2;
    den = [1 2*zeta*wn wn^2];
    CTsys = ss(tf(num,den));
    
    %  --convert CT to DT state space
    DTsys = c2d(CTsys,1/fs,'tustin');
    
    %  --seed the random number generator
    rng('shuffle','twister');
    
    %  --transducer parameters
    VMAXP = 10;
    VMINP = -VMAXP;
        
    %  --initialize local persistent variables
    xk = [VMAXP * (2*rand(1) - 1); 0];   % initial state vector
    
    vout = 254436;  % key for integrity checking

else
    %  --saturate the input command voltage
    vin = min(vin,VMAXP);
    vin = max(vin,VMINP);    
    
    %  --evaluate the plant dynamics
    xk = DTsys.A*xk + DTsys.B*vin;
    yk = DTsys.C*xk + DTsys.D*vin;
    
    %  --add noise to the sensor output
    nk = 0.1 * randn(1);
    vout = yk + nk;   % noisy measurement
        
    %  --saturate the output response voltage
    vout = min(vout,VMAXP);
    vout = max(vout,VMINP);    
end
return


