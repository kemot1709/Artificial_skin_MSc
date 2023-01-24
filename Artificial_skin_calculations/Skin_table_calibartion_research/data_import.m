function data = data_import(filename)

% Returned data format:
% x.obj_params      - raw params of object written in string
% x.weight          - actual weight of object
% x.area            - area of contact of object
% x.idle_read[]     - read in all states with no load
% x.work_read[i][]  - read in all states with load (object),
%                       i values for i test samples
% x.work_avg[]      - average of all samples value with load for all states
% x.work_max[]      - maximum of all samples value with load for all states
% x.work_min[]      - minimum of all samples value with load for all states

if class(filename) == "string"
    filename = filename{1};
end

if class(filename) ~= "char"
    data = 0;
    return;
end

filename_values = str2double(regexp(filename, '\d+', 'match'));
filename_dashes = strfind(filename, '-');
raw_data = readmatrix(filename, 'DecimalSeparator', '.');
raw_data_size = size(raw_data);

data.obj_params = extractAfter(filename, filename_dashes(1));
data.weight = filename_values(1);
data.area = 0; % TODO
data.idle_read = raw_data(1, :);
data.work_read = raw_data(2:raw_data_size(1), :);
data.work_avg = mean(data.work_read);
data.work_max = max(data.work_read);
data.work_min = min(data.work_read);
