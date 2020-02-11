%% Initialize serial port
delete all

s = serial("COM7", "BaudRate", 9600);
fclose(s)
fopen(s)

%% Read serial values
for i=1:500
    msg = fscanf(s);
    [ir, us, pot] = parse_message(msg)
end

function [ir, us, pot] = parse_message(msg)
    split_vals = strsplit(msg,',');
    
    ir = cell2mat(split_vals(1));
    us = cell2mat(split_vals(2));
    pot = cell2mat(split_vals(3));
end