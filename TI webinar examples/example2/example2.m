function example2

%  --define global symbols to pass to the interrupt handler
global DATA_BUFFER BUFLEN BUFFER_FULL START_DAQ

BUFLEN = 500;  % number of samples to collect
BUFFER_FULL = false;
DATA_BUFFER = zeros(BUFLEN,4);
START_DAQ = false;


%  --configure the hardware
MSP432_set_system_clock(48);    % 48 MHz

timer_clk_hz = 32768;   % frequency of TIMERA clock (Hz)
MSP432_set_ADC_timer_clock(timer_clk_hz);

fs = 1000;      % desired ADC sample rate (Hz)

timera_period = round(timer_clk_hz / fs);
MSP432_set_ADC_timer_period(timera_period);


%  --finally enable TIMERA and the ADC data acquisition
MSP432_enable_interrupts(true);


%  --turn off user LED's and switches at startup
MSP432_set_LED_state('LED1',0);     % turn LED1 off
MSP432_set_LED_state('LED2',0);     % turn LED2 off
MSP432_set_switch_state('S1',false);    % turn S1 off
MSP432_set_switch_state('S2',false);    % turn S2 off


%  --start the infinite background loop
while(~MSP432_test_for_reset)
    
    %  --check the S1 switch (enable or disable data acquisition)
    if (MSP432_get_switch_state('S1') == true)
        %  --first turn off switch S1
        MSP432_set_switch_state('S1',false);
        
        %  --get the current LED1 state (0=off, 1=on)
        led1state = MSP432_get_LED_state('LED1');
        
        %  --toggle the state of LED1
        led1state = ~led1state;

        MSP432_set_LED_state('LED1',led1state);
        MSP432_set_LED_state('LED2',0);     % turn LED2 off
        
        %  --set the run mode for the interrupt handler
        START_DAQ = led1state;
    end
    
    
    %  --toggle LED2 while collecting data
    if START_DAQ
        led2state = MSP432_get_LED_state('LED2');
        if (led2state == 0)
            led2state = 3;  % blue
        else
            led2state = 5;  % cyan
        end
        MSP432_set_LED_state('LED2',led2state);
    end
         
    
    %  --check to see if we have a full buffer of data
    if BUFFER_FULL
        %  --buffer is full, so write the data to the workspace
        MSP432_write_data_to_workspace('data',DATA_BUFFER);
                
        BUFFER_FULL = false;  % prevent continuous writing to workspace
        START_DAQ = false;    % disable data acquisition
        MSP432_set_LED_state('LED1',0);  % turn off LED1
        MSP432_set_LED_state('LED2',2);  % turn LED2 (green) on to indicate done
    end
    
    %  --wait before repeating the loop
    pause(0.25)    % seconds
end

return
