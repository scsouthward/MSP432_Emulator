function ADC14_IRQHandler
%  This function handles interrupts from the ADC14 analog-to-digital
%  converter (emulated) on the MSP432 microcontroller.  When an ADC sample
%  conversion is complete, the ADC hardware issues an interrupt to the
%  processor, which activates this function.

global DATA_BUFFER BUFLEN BUFFER_FULL START_DAQ VMAXP VMINP
global NUMLPF DENLPF RUN_MODE PWM_ENABLE NUMCONTROL DENCONTROL

persistent count lpf_in lpf_out control_in control_out
persistent refcount refstate Vref LPF_ORDER CONTROL_ORDER

if isempty(count)  % Initialize only on the very first time
    %  --local counters
    count = 1;
    refcount = 0;
    refstate = 0;
    Vref = 0;
    %  --difference equation memory vectors
    LPF_ORDER = length(DENLPF);
    lpf_in = zeros(1,LPF_ORDER);
    lpf_out = zeros(1,LPF_ORDER);
    
    CONTROL_ORDER = length(DENCONTROL);
    control_in = zeros(1,CONTROL_ORDER);
    control_out = zeros(1,CONTROL_ORDER);   
end


%  --read the ADC samples (14-bit ADC -> 2^14=16384)
adc_sample = MSP432_get_ADC_value;  % raw value will be 0 to 16383

%  --scale the raw ADC samples to voltage units
Vsens = 20 * (adc_sample / 16384) - 10;  % this is the feedback measurement


switch RUN_MODE
    case 0      % Idle mode
        %  --disable the PWM output (0 volts)
        Vcontrol = 0;
        
        %  --reset all filter memory to zero
        lpf_in = zeros(1,LPF_ORDER);
        lpf_out = zeros(1,LPF_ORDER);
        control_in = zeros(1,CONTROL_ORDER);
        control_out = zeros(1,CONTROL_ORDER);

        %  --force all filter coefficient vectors to be column vectors
        NUMLPF = NUMLPF(:);
        DENLPF = DENLPF(:);
        NUMCONTROL = NUMCONTROL(:);
        DENCONTROL = DENCONTROL(:);
        
    case 1      % System Identification mode
        %  --generate a random noise sequence
        Vrand = (2*VMAXP)*rand(1) - VMAXP;   % uniform distribution from -10V to +10V
        
        %  --shift the new input sample to the input filter memory
        lpf_in = [Vrand, lpf_in(1:LPF_ORDER-1)];
        
        %  --generate the next filtered output sample
        if (LPF_ORDER == 1)
            %  --no need to process the denominator terms
            Vcontrol = lpf_in * NUMLPF;
        else
            Vcontrol = lpf_in * NUMLPF - lpf_out(1:end-1) * DENLPF(2:end);
        end
        
        %  --shift current output sample to the output filter memory
        lpf_out = [Vcontrol, lpf_out(1:LPF_ORDER-1)];
        
    case 2      % Feedback Control mode
        if START_DAQ
            %  --Generate a reference signal with a Finite State Machine
            refcount = refcount + 1;
            switch refstate
                case 0
                    Vref = 0;
                    if (refcount == 200), refstate = 1; refcount = 0; end
                case 1
                    Vref = 8;
                    if (refcount == 600), refstate = 2; refcount = 0; end
                case 2
                    Vref = -8;
                    if (refcount == 600), refstate = 1; refcount = 0; end
            end
        end
        
        %  --compute the error signal
        error = Vref - Vsens;
        
        
        %  --evaluate the control law (general z-domain transfer function)
        %             -----------------
        %   error(z)  | NUMCONTROL(z) |  Vcontrol(z)
        %  ---------->| ------------- |-------------->
        %             | DENCONTROL(z) |
        %             -----------------
        
        %  --shift the new input sample to the input filter memory
        control_in = [error, control_in(1:CONTROL_ORDER-1)];
        
        %  --generate the next controller output sample
        if (CONTROL_ORDER == 1)
            %  --no need to process the denominator terms
            Vcontrol = control_in * NUMCONTROL;
        else
            Vcontrol = control_in * NUMCONTROL - control_out(1:end-1) * DENCONTROL(2:end);
        end
        
        %  --shift current output sample to the output filter memory
        control_out = [Vcontrol, control_out(1:CONTROL_ORDER-1)];
end

%  --saturate the control voltage
Vcontrol = min(VMAXP,Vcontrol);    % saturate at +10V
Vcontrol = max(VMINP,Vcontrol);    % saturate at -10V

%  --compute the PWM duty cycle for this control voltage
duty_cycle = (Vcontrol + VMAXP) / (2*VMAXP);  % 50% duty_cycle = 0 volts

%  --convert the duty cycle to a PWM count
if PWM_ENABLE
    pwm_count = round(MSP432_get_PWM_base_period * duty_cycle);
else
    %  --when PWM is disabled, output 50% duty cycle, which is 0 volts
    pwm_count = round(MSP432_get_PWM_base_period / 2);
end

%  --output the PWM signal
MSP432_set_PWM_count(pwm_count);


%  --only save data when enabled and when the buffer is not full
if (START_DAQ && ~BUFFER_FULL)
    %  --save the raw ADC data to the buffer
    DATA_BUFFER(count,:) = [Vsens, Vcontrol, Vref];
    
    if (count == BUFLEN)
        %  --reset counter and notify for a full buffer
        count = 1;
        BUFFER_FULL = true;
    else
        %  --increment counter for the next sample
        count = count + 1;
        BUFFER_FULL = false;
    end
end

return
