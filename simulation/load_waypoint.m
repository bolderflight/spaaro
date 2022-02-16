function flightplan = load_waypoint (flightplan, waypoint_file)
fid = fopen(waypoint_file);
j=1;
tline = fgetl(fid);
while ischar(tline)
    line(j,:) = convertCharsToStrings(tline);
    tline = fgetl(fid);
    j=j+1;
end
waypoint_str = line(3:end,:);

for i = 1:size(waypoint_str,1)
    waypoint(i,:) = strsplit(waypoint_str(i,1),'\t');
end

waypoint = str2double(waypoint);
% Only apply for MAV_FRAME = 3
for i=1:size(waypoint,1)
    flightplan(i).autocontinue = boolean(waypoint(i,12));
    flightplan(i).frame = uint8(waypoint (i,3));
    flightplan(i).cmd = uint16(waypoint (i,4));
    flightplan(i).param1 = single(waypoint(i,5));
    flightplan(i).param2 = single(waypoint(i,6));
    flightplan(i).param3 = single(waypoint(i,7));
    flightplan(i).param4 = single(waypoint(i,8));
    if uint16(waypoint (i,4)) == 16
        flightplan(i).x = int32(waypoint(i,9)*10E6);
        flightplan(i).y = int32(waypoint(i,10)*10E6);
        flightplan(i).z = single(waypoint(i,11));
    else
        flightplan(i).x = int32(waypoint(i,9));
        flightplan(i).y = int32(waypoint(i,10));
        flightplan(i).z = single(waypoint(i,11));
    end
    
end
end