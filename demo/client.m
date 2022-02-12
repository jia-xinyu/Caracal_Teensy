clc
clear

%% init message
% --- 48 bytes ---
[tau_a_des, tau_b_des, tau_c_des] = deal(zeros(3,1,'single'));
flags = zeros(3,1,'int32');
% checksum = zeros(3,1,'int32');

% --- 108 bytes ---
[q_a, q_b, q_c, qd_a, qd_b, qd_c, tau_a, tau_b, tau_c] = deal(zeros(3,1,'single')); 
% flags = zeros(3,1,'int32');
% checksum = zeros(3,1,'int32');

tx_size = 48;
rx_size = 108;

%% init UDP
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

%% controller parameters
Kp = 40; Kd = 3; tau_limit = 30;
q_data = 0; qd_data = 0;

%% send command and receive data
for i = 1:10000
    % --- controller ---
    q_des = 0; qd_des = 0;
	% q_des = (pi/2) * sin((2*M_PI/2)*i);
	% qd_des = (pi/2) * (pi) * cos((2*pi/2)*i);
    tau_des = Kp * (q_des-q_data) + Kd * (qd_des-qd_data);
    tau_des = min([max([tau_des, -tau_limit]), tau_limit]);
    % disp(tau_des);
    
    % --- input command ---
    tau_a_des(1) = tau_des;
    
    % --- pack ---
    joint_command = single([tau_a_des; tau_b_des; tau_c_des; flags;]);
    joint_command = reshape(joint_command, [12,1]);
    % whos joint_command  % check size

    udp_tx = typecast(joint_command,'uint8');
    fwrite(udp_m, udp_tx, 'uint8');

    if udp_m.BytesAvailable > 0
        udp_rx = fread(udp_m, rx_size,'uint8');
        udp_rx = typecast(uint8(udp_rx),'single');  % double -> uint8 -> single
        
        % --- unpack ---
        % joint_data = single([q_a; q_b; q_c; qd_a; qd_b; qd_c; tau_a; tau_b; tau_c;]);
        % whos joint_data  % check size
        joint_data = reshape(udp_rx, [27,1]);
        q_a = joint_data(1:3);
        q_b = joint_data(4:6);
        q_c = joint_data(7:9);
        qd_a = joint_data(10:12);
        qd_b = joint_data(13:15);
        qd_c = joint_data(16:18);
        tau_a = joint_data(19:21);
        tau_b = joint_data(22:24);
        tau_c = joint_data(25:27);
    end
    
    % --- output data ---
    q_data = q_a(1); qd_data = qd_a(1);
end

%% close
fclose(udp_m);
delete(udp_m);
clear ip_local ip_remote port_local port_remote;
