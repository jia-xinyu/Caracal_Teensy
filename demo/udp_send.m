clc
clear

%% message
% --- 48 bytes ---
tau_a_des = zeros(3,1,'single');
tau_b_des = zeros(3,1,'single');
tau_c_des = zeros(3,1,'single');
flags = zeros(3,1,'int32');
% checksum = zeros(3,1,'int32');  % be careful of int

% --- 108 bytes ---
q_a = zeros(3,1,'single'); 
q_b = zeros(3,1,'single');
q_c = zeros(3,1,'single');
qd_a = zeros(3,1,'single');
qd_b = zeros(3,1,'single');
qd_c = zeros(3,1,'single');
tau_a = zeros(3,1,'single');
tau_b = zeros(3,1,'single');
tau_c = zeros(3,1,'single');
% flags = zeros(3,1,'int32');
% checksum = zeros(3,1,'int32');

% check size
% whos joint_command
% whos joint_data

tx_size = 48;
rx_size = 108;

%% init
% local IP (keep first three sections same as remote IP)
ip_local = '192.168.137.178'; port_local = 8080;

% remtoe IP
ip_remote = '192.168.137.177'; port_remote = 8080;

udp_m = udp(ip_remote, 'RemotePort', port_remote, 'LocalPort', port_local);
udp_m.OutputBufferSize = tx_size;
udp_m.InputBufferSize = rx_size;
udp_m.TimeOut = 5;  % wait for 5 s when the buffer is empty

fopen(udp_m);
% for the error "Unsuccessful open: Address already in use (Bind failed)"
% change a new local port

%% first bind
bind_tx = 'Hello from PC !';
fprintf(udp_m, bind_tx);  % test tx

if udp_m.BytesAvailable > 0
    fprintf('Receive data successfully !');
    bind_rx = fscanf(udp_m, 'char');                % test rx
    disp(class(bind_rx)); disp(size(bind_rx));      % data type and size
end

% Notes:
% char: fprint(send) / fscanf(recv)
% number: fwrite(send) / fread(recv)

%% input command
tau_a_des = [0;0;0;];

%% send command and receive data
for i = 1:10000
    % --- pack ---
    joint_command = single([tau_a_des; tau_b_des; tau_c_des; flags;]);
    joint_command = reshape(joint_command, [12,1]);
    % whos joint_command

    udp_tx = typecast(joint_command,'uint8');
    fwrite(udp_m, udp_tx, 'uint8');

    if udp_m.BytesAvailable > 0
        udp_rx = fread(udp_m, rx_size,'uint8');
        udp_rx = typecast(uint8(udp_rx),'single');  % double -> uint8 -> single
        
        % --- unpack ---
        % joint_data = single([q_a; q_b; q_c; qd_a; qd_b; qd_c; tau_a; tau_b; tau_c;]);
        joint_data = reshape(udp_rx, [27,1]);
        q_a = joint_data(1:3,1);
        q_b = joint_data(4:6,1);
        q_c = joint_data(7:9,1);
        qd_a = joint_data(10:12,1);
        qd_b = joint_data(13:15,1);
        qd_c = joint_data(16:18,1);
        tau_a = joint_data(19:21,1);
        tau_b = joint_data(22:24,1);
        tau_c = joint_data(25:27,1);
           
        joint_a = q_a(1);
        disp(joint_a);
    end
end

%% close
fclose(udp_m);
delete(udp_m);
clear ip_local ip_remote port_local port_remote;
