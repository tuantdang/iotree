clear, clc, close all;
N = 16;
%files = string(N);
%folder = "D:/longrun/";
folder = "longrun/";
test = "8";
c = 5.1002e-10;

isPlotAll = 0;

f = [1e4 2e4 3e4 4e4 5e4 6e4 7e4 8e4 9e4 1e5 2e5 3e5 4e5 5e5 6e5];
figure;
f1 = 0;
f10 = 0;
from = 0;
to = 0;
for i = 1:N
    fileName = folder + test + "/f" + num2str(i) + ".csv";
    table = readtable(fileName);
    time = table2array(table(:,1))';
    time = time/(60*60);
    mag = table2array(table(:,4))';
    imp = (mag.^-1)/c;
    
    %subplot(1,1,i);
    from = 1;
    to = length(time);
    
    if i == 1
        f1 = imp;
    elseif i == 2
        f2 = imp;
    end
        
    to = 0;
    for j = 1:length(time)
        if time(j) >= 5.9
            to = j;
            break;
        end
    end
    
    to = length(time);
    if isPlotAll == 1
        subplot(4,4,i);
        plot(time(from:to), imp(from:to));
        grid on;            
        title("F" + num2str(i));
        xlabel('hours');
        ylabel('Ohm'); 
    end
    %}
    break;
end

imp1 = imp;
time1 = time;

test = "7";
startFreq = 3;
for i = startFreq:N
    fileName = folder + test + "/f" + num2str(i) + ".csv";
    table = readtable(fileName);
    time = table2array(table(:,1))';
    time = time/(60*60);
    mag = table2array(table(:,4))';
    imp = (mag.^-1)/c;
    
    %subplot(1,1,i);
    from = 1;
    to = length(time);
    
    if i == 1
        f1 = imp;
    elseif i == 2
        f2 = imp;
    end
        
    to = 0;
    for j = 1:length(time)
        if time(j) >= 5.9
            to = j;
            break;
        end
    end
    
    to = length(time);
    if isPlotAll == 1
        subplot(4,4,i);
        plot(time(from:to), imp(from:to));
        grid on;            
        title("F" + num2str(i));
        xlabel('hours');
        ylabel('Ohm'); 
    end
    %}
    break;
end

imp2 = imp;
time2 = time + time1(length(time1));

imp = [imp1 imp2];
%imp = imp / max(imp);
time = [time1 time2];
%figure;

len = length(time);
stop = 0;
for i = 1:len
    if time(i) > 100
        stop = i;
        break
    end
end

time = time(1:stop); time = time*1.4
imp = imp(1:stop);
%imp = imp.^-1;

yrange = [2.55e-6 2.7e-6];
if isPlotAll == 0
    %plot(time(from:to), f1(from:to));
    plot(time*1.2, 1./imp);
    xlim([125 150]);
%     ylim(yrange);
    %grid on;
    %xlabel('Time (hour)');
    %ylabel('Impedance');
end

