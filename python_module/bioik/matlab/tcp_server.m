while true
    
    clear % remove existing tcpip object
    
    TCP_IP = '127.0.0.1';
    TCP_PORT = 5005;

    t=tcpip(TCP_IP, TCP_PORT, 'NetworkRole', 'server');

    % Set size of receiving buffer, if needed. 
    set(t, 'InputBufferSize', 1);

    % Open connection to the server. 
    fopen(t);
    
    DataReceived = 'o';
    nb_received = 0;

    display('connected')

    while ~strcmp( DataReceived, 'c') % wait for close message
        
        while get(t, 'BytesAvailable') > 0 % wait for new bytes available
            DataReceived = fscanf(t);
            nb_received = nb_received + 1;
            run_one_ik
            fwrite(t, num2str(mod(nb_received,4))); % send acknowledge
        end
    end
    
    nb_received
    
    display('disconnected')
    % Disconnect and clean up the server connection. 
    fclose(t);
    delete(t);
    pause(1)
end