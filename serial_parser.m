%% Initialize serial port
s = serial("COM3", "BaudRate", 9600);
fclose(s)
fopen(s)

%%
%robot = HebiLookup.newGroupFromNames('RoboDutchman', {'joint1', 'joint2', 'joint3'});
%cmd = CommandStruct();
commandwindow
while 1
    msg = fscanf(s);
    [irp, usp, potp, but] = parse_message(msg);
    cmd.position = [irp usp potp];
    robot.set(cmd);
end

function [ir, us, pot, but] = parse_message(msg)
    persistent irp usp potp butp
    split_vals = strsplit(msg,',');
    try
        % Parse values
        irp = cell2mat(split_vals(1));
        usp = cell2mat(split_vals(2));
        potp = cell2mat(split_vals(3));
        butp = cell2mat(split_vals(4));

        % Map to 2*pi range
        irp = str2double(irp) * 2 * pi / 60;
        usp = str2double(usp) * 2 * pi / 120;
        potp = 2*pi - str2double(potp) * 2 * pi / 1023;
        butp = str2double(butp);
        disp("ir: " + irp + "   us: " + usp + ...
            "  pot: " + potp + " butp: " + butp)
    catch
    end
    ir = irp;
    us = usp;
    pot = potp;
    but = butp;
end