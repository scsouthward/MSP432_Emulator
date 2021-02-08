function ADC14_IRQHandler
%  This function handles interrupts from the ADC14 analog-to-digital
%  converter (emulated) on the MSP432 microcontroller.  When an ADC sample
%  conversion is complete, the ADC hardware issues an interrupt to the
%  processor, which activates this function.
%
%  This function can NOT have any inputs or outputs, so all signals must be
%  passed through global variables.
%
%  When exiting this function, all local variables are cleared. If you need
%  to retain the value in a local variable, it must be declared as
%  persistent (the equivalent in C would be a static declaration).
%
%  Global variables cannot be declared as persistent

global DATA_BUFFER BUFLEN BUFFER_FULL START_DAQ

persistent count
if isempty(count), count = 1; end  % Initialize only on the very first time


%  --read the ADC samples (14-bit ADC -> 2^14=16384)
adc_sample = MSP432_get_ADC_value([1,2]);  % raw value will be 0 to 16383

%  --scale the raw ADC sample to voltage units
Vsens = 20 * (adc_sample / 16384) - 10;


%  --only save data when enabled and when the buffer is not full
if (START_DAQ && ~BUFFER_FULL)
    %  --save the raw ADC data to the buffer
    DATA_BUFFER(count,:) = [adc_sample, Vsens];
    
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
