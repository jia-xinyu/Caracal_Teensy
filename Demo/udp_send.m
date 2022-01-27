clc
clear

%% message
% --- 48 bytes ---
tau_a_des = zeros(1,3,'single');
tau_b_des = zeros(1,3,'single');
tau_c_des = zeros(1,3,'single');
flags = zeros(1,3,'int32');
% checksum = zeros(1,3,'int32');  % be careful of int

% --- 108 bytes ---
q_a = zeros(1,3,'single'); 
q_b = zeros(1,3,'single');
q_c = zeros(1,3,'single');
qd_a = zeros(1,3,'single');
qd_b = zeros(1,3,'single');
qd_c = zeros(1,3,'single');
tau_a = zeros(1,3,'single');
tau_b = zeros(1,3,'single');
tau_c = zeros(1,3,'single');
% flags = zeros(1,3,'int32');
% checksum = zeros(1,3,'int32');

% check size
% whos joint_command
% whos joint_data

tx_size = 48;
rx_size = 108;

%% init
% local IP (keep first three sections same as remote IP)
ip_local = '192.168.137.178'; port_local = 5084;

% remtoe IP
ip_remote = '192.168.137.177'; port_remote = 8080;

udp_m = udp(ip_remote, 'RemotePort', port_remote, 'LocalPort', port_local);
udp_m.OutputBufferSize = tx_size;
udp_m.InputBufferSize = rx_size;
udp_m.TimeOut = 5;  % wait for 5 s when the buffer is empty

fopen(udp_m);
% for the error "Unsuccessful open: Address already in use (Bind failed)"
% change a new local port

%% test bind
bind_tx = 'Hello from PC !';
fprintf(udp_m, bind_tx);  % test tx

if udp_m.BytesAvailable > 0
    fprintf('Receive data successfully !');
    bind_rx = fscanf(udp_m, 'char');  % test rx
    disp(class(bind_rx));  % data type
    disp(size(bind_rx));  % data size
end

% Notes:
% char: fprint(send) / fscanf(recv)
% number: fwrite(send) / fread(recv)

%% input command
tau_a_des = [40,50,50];
tau_b_des = [30,50,50];
tau_c_des = [10,50,50];

%% send command and receive data
% --- pack ---
joint_command = single([tau_a_des, tau_b_des, tau_c_des, flags]);
udp_tx = reshape(joint_command, 12, 1);
% whos udp_tx

fwrite(udp_m, udp_tx, 'char');

if udp_m.BytesAvailable > 0
    udp_rx = fread(udp_m, rx_size);
    data = str2num(char(udp_rx(1:end-1)'));
    disp(data)
end

% --- unpack ---
joint_data = single([q_a; q_b; q_c; qd_a; qd_b; qd_c; tau_a; tau_b; tau_c;]);
joint_data = reshape(joint_data, 27, 1);

%% close
fclose(udp_m);
delete(udp_m);
clear ip_local port_local ip_remote port_remote;
