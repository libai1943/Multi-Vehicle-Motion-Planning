% ==============================================================================
% MATLAB Source Codes for "Incrementally constrained dynamic optimization: A computational
% framework for lane change motion planning of connected and automated vehicles".

% ==============================================================================

%   Copyright (C) 2018 Bai Li
%   Useers must cite the following article when utilizing these source codes to produce new contributions.
%   Bai Li et al., ""Incrementally constrained dynamic optimization: A computational framework for lane change
%   motion planning of connected and automated vehicles",  Submitted to
%   Journal of Intelligent Transportation Systems: Technology, Planning, and Operations.

% ==============================================================================

% If there are inquiries, feel free to contact libaioutstanding@163.com

% 2018.07.11
% ==============================================================================

clear all
close all
clc

NV = 12;
global gapmax
gapmax = 1;

% Case Configuration Setup
[nA1,nA2,nA3,nA4] = randfixedsum(NV);
[nA11,nA12,nA13,nA14] = randfixedsum(NV);

delete('N1');
fid = fopen('N1', 'w');
fprintf(fid,'1  %g ', nA11);
fclose(fid);
delete('N2');
fid = fopen('N2', 'w');
fprintf(fid,'1  %g ', nA12);
fclose(fid);
delete('N3');
fid = fopen('N3', 'w');
fprintf(fid,'1  %g ', nA13);
fclose(fid);
delete('N4');
fid = fopen('N4', 'w');
fprintf(fid,'1  %g ', nA14);
fclose(fid);

delete('OLD1');
fid = fopen('OLD1', 'w');
fprintf(fid,'1  %g ', nA1);
fclose(fid);
delete('OLD2');
fid = fopen('OLD2', 'w');
fprintf(fid,'1  %g ', nA2);
fclose(fid);
delete('OLD3');
fid = fopen('OLD3', 'w');
fprintf(fid,'1  %g ', nA3);
fclose(fid);
delete('OLD4');
fid = fopen('OLD4', 'w');
fprintf(fid,'1  %g ', nA4);
fclose(fid);

PXPY_generater(nA1,nA2,nA3,nA4);
temp = randperm(NV);

A1 = temp(1,1:nA11);
A2 = temp(1,(nA11+1):(nA11+nA12));
A3 = temp(1,(nA11+nA12+1):(nA11+nA12+nA13));
A4 = temp(1,(nA11+nA12+nA13+1):(nA11+nA12+nA13+nA14));

delete('A1');
fid = fopen('A1', 'w');
for ii = 1:length(A1)
    fprintf(fid,'%g ', A1(ii));
end
fclose(fid);
delete('A2');
fid = fopen('A2', 'w');
for ii = 1:length(A2)
    fprintf(fid,'%g ', A2(ii));
end
fclose(fid);
delete('A3');
fid = fopen('A3', 'w');
for ii = 1:length(A3)
    fprintf(fid,'%g ', A3(ii));
end
fclose(fid);
delete('A4');
fid = fopen('A4', 'w');
for ii = 1:length(A4)
    fprintf(fid,'%g ', A4(ii));
end
fclose(fid);


!ampl r00.run
!ampl r01.run
AAAA = num2str(0);
lane_change_video_generation;

ii_ind = 1;
while (ii_ind <= 10)
    delete('libai');
    fid = fopen('libai', 'w');
    fprintf(fid,'1 %g;', ii_ind);
    fclose(fid);
    
    !ampl r02.run
    AAAA = num2str(ii_ind);
    lane_change_video_generation;
    flag = collision_checker;
    
    if (flag == 0)
        ii_ind = 11;
    else
        ii_ind = ii_ind + 1;
    end
end

flag = collision_checker;
if (flag == 1)
    error('Solution Process Ends with a Failure');
end
AAAA = 'Final_Result';
lane_change_video_generation;