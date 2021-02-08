function example1
%  This is an example program for the MSP432 emulator

%  --initialize the hardware at startup

MSP432_set_LED_state('LED1',0);         % turn LED1 off
MSP432_set_LED_state('LED2',0);         % turn LED2 off
MSP432_set_switch_state('S1',false);    % turn S1 off
MSP432_set_switch_state('S2',false);    % turn S2 off


%  --start the infinite background loop

while(~MSP432_test_for_reset)
    
    %  --read the S1 and S2 switches
    S1state = MSP432_get_switch_state('S1');
    S2state = MSP432_get_switch_state('S2');
    
    %  --read the LED1 and LED2 states
    LED1state = MSP432_get_LED_state('LED1');
    LED2state = MSP432_get_LED_state('LED2');
    
    %  --switch LED1 based on S1
    if (S1state == true)
        %  --toggle LED1
        MSP432_set_LED_state('LED1',~LED1state);
        
        %  --user has turned S1 on, so now turn it off
        MSP432_set_switch_state('S1',false);
    end

    %  --switch LED2 based on S2
    if (S2state == true)
        %  --increment LED2
        LED2state = LED2state + 1;
        if (LED2state > 7), LED2state = 0; end
        MSP432_set_LED_state('LED2',LED2state);
        
        %  --user has turned S2 on, so now turn it off
        MSP432_set_switch_state('S2',false);
    end
    
    %  --wait before repeating the loop
    pause(0.1)    % seconds
end

return
