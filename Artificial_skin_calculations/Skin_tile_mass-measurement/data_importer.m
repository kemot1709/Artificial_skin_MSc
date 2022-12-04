function data = data_importer(filename)
M = readmatrix(filename,'DecimalSeparator',',');

M=M(2:end,1:7);
M(1,1)=5;
M(1,2:end)=3000;
M(:,2:end) = sort(M(:,2:end),2);

M_d=M(:,2:end);

M=[M mean(M_d,2)];
M=[M (sum(M_d,2)-min(M_d,[],2)-max(M_d,[],2))/(size(M_d,2)-2)];


data=M;