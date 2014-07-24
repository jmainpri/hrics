% Create TCP/IP object 't'. Specify server machine and port number. 
% t = tcpip('www.EXAMPLE_WEBSITE.com', 80); 

while true
    
    clear
    
    DataReceived = 'o';
    nb_received = 0;

    t=tcpip('127.0.0.1', 5005, 'NetworkRole', 'server');

    % Set size of receiving buffer, if needed. 
    set(t, 'InputBufferSize', 1);

    % Open connection to the server. 
    fopen(t);

    display('connected')

    while ~strcmp( DataReceived, 'c')

        while get(t, 'BytesAvailable') > 0
%             t.BytesAvailable 
            DataReceived = fscanf(t);
            nb_received = nb_received + 1;
            fwrite(t, num2str(mod(nb_received,4)));
        end
    end
    
    nb_received
    
    display('disconnected')
    % Disconnect and clean up the server connection. 
    fclose(t);
    delete(t);
    pause(1)
end