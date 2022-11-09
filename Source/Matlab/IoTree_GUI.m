function varargout = IoTree_GUI(varargin)
% IOTREE_GUI MATLAB code for IoTree_GUI.fig
%      IOTREE_GUI, by itself, creates a new IOTREE_GUI or raises the existing
%      singleton*.
%
%      H = IOTREE_GUI returns the handle to a new IOTREE_GUI or the handle to
%      the existing singleton*.
%
%      IOTREE_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IOTREE_GUI.M with the given input arguments.
%
%      IOTREE_GUI('Property','Value',...) creates a new IOTREE_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before IoTree_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to IoTree_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help IoTree_GUI

% Last Modified by GUIDE v2.5 09-Apr-2021 16:29:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @IoTree_GUI_OpeningFcn, ...
    'gui_OutputFcn',  @IoTree_GUI_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before IoTree_GUI is made visible.
function IoTree_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to IoTree_GUI (see VARARGIN)

% Choose default command line output for IoTree_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes IoTree_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

global M;
global vFreq;
global vReal;
global vImg;
global vMag;
global vImp ;
global time;

global vFreq2;
global vReal2;
global vImg2;
global vMag2;
global vImp2;
global counter;
global bLoop;

M = 100;
vFreq = zeros(1, M);
vReal = zeros(1, M);
vImg = zeros(1, M);
vMag = zeros(1, M);
vImp = zeros(1, M);
time = zeros(1, M);

vFreq2 = zeros(1, M);
vReal2 = zeros(1, M);
vImg2 = zeros(1, M);
vMag2 = zeros(1, M);
vImp2 = zeros(1, M);
counter = 0;

global bPortOpened;
if bPortOpened == 1
    set(handles.btnConnect, 'Enable', 'off');
    set(handles.btnDisconnect, 'Enable', 'on');
else
    set(handles.btnConnect, 'Enable', 'on');
    set(handles.btnDisconnect, 'Enable', 'off');
end
%Detect available com number and assign to GUI
defaultPort = 1;
lComs = seriallist();
s = size(lComs);
if s(2) == 1   
    defaultPort = str2num(extractBetween(lComs(1), 4,4));
elseif s(2) > 1
    defaultPort = str2num(extractBetween(lComs(1), 4,4)) + 1;
end
 %set(handles.drbCom, 'Value', defaultPort);
 set(handles.drbCom, 'Value', 3);
 set(handles.drbFunc, 'Value', 6);
 set(handles.drbChannel, 'Value', 3);
 %Date time format
 datetime.setDefaultFormats('default','yyyy-MM-dd hh-mm-ss')



% --- Outputs from this function are returned to the command line.
function varargout = IoTree_GUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function txtCtlReg_Callback(hObject, eventdata, handles)
% hObject    handle to txtCtlReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtCtlReg as text
%        str2double(get(hObject,'String')) returns contents of txtCtlReg as a double


% --- Executes during object creation, after setting all properties.
function txtCtlReg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtCtlReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txtStatusReg_Callback(hObject, eventdata, handles)
% hObject    handle to txtStatusReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtStatusReg as text
%        str2double(get(hObject,'String')) returns contents of txtStatusReg as a double


% --- Executes during object creation, after setting all properties.
function txtStatusReg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtStatusReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txtStartFreqReg_Callback(hObject, eventdata, handles)
% hObject    handle to txtStartFreqReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtStartFreqReg as text
%        str2double(get(hObject,'String')) returns contents of txtStartFreqReg as a double


% --- Executes during object creation, after setting all properties.
function txtStartFreqReg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtStartFreqReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txtIncReg_Callback(hObject, eventdata, handles)
% hObject    handle to txtIncReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtIncReg as text
%        str2double(get(hObject,'String')) returns contents of txtIncReg as a double


% --- Executes during object creation, after setting all properties.
function txtIncReg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtIncReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txtNumSamplesReg_Callback(hObject, eventdata, handles)
% hObject    handle to txtNumSamplesReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtNumSamplesReg as text
%        str2double(get(hObject,'String')) returns contents of txtNumSamplesReg as a double


% --- Executes during object creation, after setting all properties.
function txtNumSamplesReg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtNumSamplesReg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnConnect.
function btnConnect_Callback(hObject, eventdata, handles)
% hObject    handle to btnConnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sPort;
global bPortOpened;
try
    if (bPortOpened == 1)
        msgbox("Serial Port Already Open");
        return;
    end
    port = get(handles.drbCom, 'Value');
    
    portName = sprintf("COM%d", port);
    sPort = serial(portName, 'Baudrate', 9600, 'Timeout', 60);
    fopen(sPort);
    bPortOpened = 1;
    set(handles.btnConnect, 'Enable', 'off');
    set(handles.btnDisconnect, 'Enable', 'on');
catch ex
    msgbox(ex.message);
    %fprintf("Open Serial Error: %s\n", ex.message);
end

% --- Executes on button press in btnDisconnect.
function btnDisconnect_Callback(hObject, eventdata, handles)
% hObject    handle to btnDisconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sPort;
global bPortOpened;
try
    bPortOpened = 0;
    fclose(sPort);
    delete(sPort);
catch
    msgbox("Close Serial Error");
end

set(handles.btnConnect, 'Enable', 'on');
set(handles.btnDisconnect, 'Enable', 'off');


% --- Executes on button press in btnReadRegs.
function btnReadRegs_Callback(hObject, eventdata, handles)
% hObject    handle to btnReadRegs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global bPortOpened;
global sPort;
global regCtl;
global regStatus;
if (bPortOpened == 1)
    try
        %fwrite(sPort, 254, 'uint8');
        %fwrite(sPort, 255, 'uint8');
        fwrite(sPort, 1, 'uint8');
        %byte1 = fread(sPort, 1, 'uint8');
        %byte2 = fread(sPort, 1, 'uint8');
        %byte3 = fread(sPort, 1, 'uint8');
        
        %if byte1 == 254 && byte2 == 255 && byte3 == 1
            %Data
        regCtl = fread(sPort, 1, 'uint16');
        regStatus = fread(sPort, 1, 'uint8');
        regStartFreq = fread(sPort, 1, 'uint32');
        regInc = fread(sPort, 1, 'uint32');
        regNumSamples = fread(sPort, 1, 'uint16') - 1;
        regSettling = fread(sPort, 1, 'uint16');
        
        regStartFreq  = regStartFreq  + regInc;

        clk = 16777000;
        sF = cast((regStartFreq*(clk/4))/2^27, 'uint32');
        inc = cast((regInc*(clk/4))/2^27, 'uint32');
        fprintf("regStartFreq = %d,  regInc = %d\n", sF, inc);
        %GUI
        set(handles.txtCtlReg,'String', "0x" + dec2hex(regCtl));
        set(handles.txtStatusReg,'String', "0x" + dec2hex(regStatus));
        set(handles.txtStartFreqReg,'String', (sF));
        set(handles.txtIncReg,'String', (inc));
        set(handles.txtNumSamplesReg,'String', regNumSamples);
        set(handles.txtSettlingReg,'String', regSettling);
        %end
    catch ex
        msgbox(ex.message);
    end
end



% --- Executes on button press in btnSweep.
function btnSweep_Callback(hObject, eventdata, handles)
% hObject    handle to btnSweep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global bPortOpened;
global sPort;
global gainFactor;

global M;
global vFreq;
global vReal;
global vImg;
global vMag;
global vImp ;
global time;

global vFreq2;
global vReal2;
global vImg2;
global vMag2;
global vImp2;
global counter;
global bLoop;

if (bPortOpened == 1)    
    try
        bUseLoRa = get(handles.chkLoRa, 'Value');
        disp(bUseLoRa);
        if bUseLoRa == 0  
            if bLoop == true
                bLoop = false;            
            else
                bLoop = true;
            end
        else
            bLoop = true;
        end
        
        N = str2num(get(handles.txtNumSamplesReg, 'String'));
        startFreq = str2num(get(handles.txtStartFreqReg, 'String'));
        incFreq = str2num(get(handles.txtIncReg, 'String'));
        %fprintf("N = %d, startFreq = %d, incFreq = %d \n", N, startFreq, incFreq);
               
        %User input
        ch = get(handles.drbChannel, 'Value');
        view = get(handles.drbView, 'Value');                        
        %set(handles.btnSweep, 'Enable', 'off');
        
                       
        axes(handles.axes1);
        title('Impedance Measurement');
        cla(handles.axes1, 'reset');
        fName = "imp/" + string(datetime('now')) + ".csv";
        fileID = fopen(fName,'w');
        fprintf(fileID, "Frequency; Real; Image; Magnitude; Impedance\n");      

        while bLoop
            counter = counter + 1;
            if bUseLoRa == 0
                fwrite(sPort, 2, 'uint8');                                           
            end            
            
            if counter <= M
                time(counter) = counter;
            else
                time = circshift(time, -1);
                time(M)= counter;
            end
            
            id = 0;
            if bUseLoRa == 1
                id = fread(sPort, 1, 'uint8');                            
            end
                           
            freq = startFreq;
            real = fread(sPort, 1, 'int16');  %2
            img = fread(sPort, 1, 'int16');   %2                       
            mag = sqrt(real*real + img*img);                        
            gainFix = 3.7111e-10; %100 KOhm
            imp = 1/(gainFix*mag);
            fprintf("%3d: id = %d, freq = %d, real = %d , img = %d, mag = %f, imp = %f \n", counter, id, freq, real, img, mag, imp);       
            if counter <= M
                vReal(counter) = real;
                vImg(counter) = img;
                vMag(counter) = mag;
                vImp(counter) = imp;
            else
                vReal = circshift(vReal, -1); vReal(M) = real;
                vImg = circshift(vImg, -1); vImg(M) = img;
                vMag = circshift(vMag, -1); vMag(M) = mag;
                vImp = circshift(vImp, -1); vImp(M) = imp;
            end
            fprintf(fileID, "%d; %d; %d; %f; %f\n", freq, real, img, mag, imp);
            
            %Freq 2            
            freq = freq + incFreq;
            real = fread(sPort, 1, 'int16');  %2
            img = fread(sPort, 1, 'int16');   %2                       
            mag = sqrt(real*real + img*img);                        
            gainFix = 3.7111e-10; %100 KOhm
            imp = 1/(gainFix*mag);
            fprintf("%3d: id = %d, freq = %d, real = %d , img = %d, mag = %f, imp = %f \n", counter, id, freq, real, img, mag, imp);          
            if counter <= M
                vReal2(counter) = real;
                vImg2(counter) = img;
                vMag2(counter) = mag;
                vImp2(counter) = imp;
            else
                vReal2 = circshift(vReal2, -1); vReal2(M) = real;
                vImg2 = circshift(vImg2, -1); vImg2(M) = img;
                vMag2 = circshift(vMag2, -1); vMag2(M) = mag;
                vImp2 = circshift(vImp2, -1); vImp2(M) = imp;
            end

            fprintf(fileID, "%d; %d; %d; %f; %f\n", freq, real, img, mag, imp);
            if view == 1
                if ch == 1
                    xlabel('Frequency');
                    ylabel('Real');
                elseif ch == 2
                    xlabel('Frequency');
                    ylabel('Image');
                elseif ch == 3
                    plot(time, vMag, time, vMag2);                     
                    xlabel('Time');
                    ylabel('Magnitude');
                    legend('15 KHz', '20 KHz');
                elseif ch == 4
                    plot(time, vImp);
                    plot(time, vImp2);
                    xlabel('Time');
                    ylabel('Impedance');
                end
            else
                if ch == 1                    
                     xlabel('Log10(Freq)');
                     ylabel('Real');
                elseif ch == 2                    
                    xlabel('Log10(Freq)');
                    ylabel('Image');
                elseif ch == 3                    
                    xlabel('Log10(Freq)');
                    ylabel('Magnitude');
                elseif ch == 4
                    %plot(log10(vFreq(1:counter)), log10(vImp(1:counter)));
                    xlabel('Log10(Freq)');
                    ylabel('Log10(Impedance)');
                end
            end %check view == 1
            drawnow;        
            if bUseLoRa == 0
                %break;
            end             
        end
        fclose(fileID);
        %set(handles.btnSweep, 'Enable', 'on');
    catch ex        
        msgbox(ex.message);        
    end
    
end


function txtCom_Callback(hObject, eventdata, handles)
% hObject    handle to txtCom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtCom as text
%        str2double(get(hObject,'String')) returns contents of txtCom as a double


% --- Executes during object creation, after setting all properties.
function txtCom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtCom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txtTemperature_Callback(hObject, eventdata, handles)
% hObject    handle to txtTemperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtTemperature as text
%        str2double(get(hObject,'String')) returns contents of txtTemperature as a double


% --- Executes during object creation, after setting all properties.
function txtTemperature_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtTemperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnReadTemperature.
function btnReadTemperature_Callback(hObject, eventdata, handles)
% hObject    handle to btnReadTemperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global bPortOpened;
global sPort;
if (bPortOpened == 1)
    try
        %fwrite(sPort, 254, 'uint8');
        %fwrite(sPort, 255, 'uint8');
        fwrite(sPort, 3, 'uint8');
        %byte1 = fread(sPort, 1, 'uint8');
        %byte2 = fread(sPort, 1, 'uint8');
        %cmd = fread(sPort, 1, 'uint8');
        val = fread(sPort, 1, 'int16');
        %if byte1 == 254 && byte2 == 255 && cmd == 3
        set(handles.txtTemperature,'String', val);
        %end
        disp(val);
    catch ex
        msgbox(ex.message);
    end
end



function txtSettlingReg_Callback(hObject, eventdata, handles)
% hObject    handle to text10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of text10 as text
%        str2double(get(hObject,'String')) returns contents of text10 as a double


% --- Executes during object creation, after setting all properties.
function txtSettlingReg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtTemperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnCtrlRegDetail.
function btnCtrlRegDetail_Callback(hObject, eventdata, handles)
% hObject    handle to btnCtrlRegDetail (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Shift(A,K); K > 0: Left; K < 0 Right

global regCtl;
func = bitand(bitshift(regCtl, -12), 255);

fName = "No Operation";
if func == 1
    fName = "Initialize with Start Frequency";
elseif func == 2
    fName = "Start Frequency Sweep";
elseif func == 3
    fName = "Increment Frequency";
elseif func == 4
    fName = "Repeat Frequency";
elseif func == 9
    fName = "Measure temperature";
elseif func == 10
    fName = "Power down";
elseif func == 11
    fName = "Standby mode";
end

range = 0;
rgnName = "No";
tmp = bitand(bitshift(regCtl, -9), 3)
if tmp == 0
    range = 1;
    rgnName = "2.0 V p-p typical";
elseif tmp == 1
    range = 4;
    rgnName = "200 mV p-p typical";
elseif tmp == 2
    range = 3;
    rgnName = "400 mV p-p typical";
elseif tmp == 3
    range = 2;
    rgnName = "1.0 V p-p typical";
end

tmp = bitand(bitshift(regCtl, -8), 1);
gain = 5;
if tmp > 0
    gain = 1;
end

reset = bitand(bitshift(regCtl, -4), 1);
exClk = bitand(bitshift(regCtl, -3), 1);
exClkStr = "Internal";
if (exClk == 1)
    exClkStr = "External";
end

msgbox(sprintf('Func (%d) : %s \nRange (%d) : %s \nGain  = %d \nReset (%d) \nClock Source (%d): %s', func, fName, range, rgnName, gain, reset, exClk, exClkStr), "Control Register");


% --- Executes on button press in btnStatusRegDetail.
function btnStatusRegDetail_Callback(hObject, eventdata, handles)
% hObject    handle to btnStatusRegDetail (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global regStatus;
func = bitand(regStatus, 7);
fName = "Invalid";
if func == 1
    fName = "Valid Temperature Measurement";
elseif func == 2
    fName = "Valid real/image data";
elseif func == 4
    fName = "Frequency Sweep Complete";
end

msgbox(sprintf('Func (%d) : %s ', func, fName), "Status Register");


% --- Executes on selection change in drbChannel.
function drbChannel_Callback(hObject, eventdata, handles)
% hObject    handle to drbChannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns drbChannel contents as cell array
%        contents{get(hObject,'Value')} returns selected item from drbChannel


% --- Executes during object creation, after setting all properties.
function drbChannel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drbChannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in drbView.
function drbView_Callback(hObject, eventdata, handles)
% hObject    handle to drbView (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns drbView contents as cell array
%        contents{get(hObject,'Value')} returns selected item from drbView


% --- Executes during object creation, after setting all properties.
function drbView_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drbView (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in drbCom.
function drbCom_Callback(hObject, eventdata, handles)
% hObject    handle to drbCom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns drbCom contents as cell array
%        contents{get(hObject,'Value')} returns selected item from drbCom


% --- Executes during object creation, after setting all properties.
function drbCom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drbCom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in drbFunc.
function drbFunc_Callback(hObject, eventdata, handles)
% hObject    handle to drbFunc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns drbFunc contents as cell array
%        contents{get(hObject,'Value')} returns selected item from drbFunc


% --- Executes during object creation, after setting all properties.
function drbFunc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drbFunc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnSetFunc.
function btnSetFunc_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetFunc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sPort;
global bPortOpened;
if bPortOpened == 0
    return;
end
func = get(handles.drbFunc, 'Value');
if func >= 5 && func <= 8
    func = func + 4;
end

fwrite(sPort, 254, 'uint8');
fwrite(sPort, 255, 'uint8');
fwrite(sPort, 16+1, 'uint8'); %CMD
fwrite(sPort, func, 'uint8'); %CMD

byte1= fread(sPort, 1, 'uint8');
byte2 = fread(sPort, 1, 'uint8');
byte3= fread(sPort, 1, 'uint8');
fprintf("%d - %d - %d \n", byte1, byte2, byte3);


% --- Executes on selection change in drbRange.
function drbRange_Callback(hObject, eventdata, handles)
% hObject    handle to drbRange (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns drbRange contents as cell array
%        contents{get(hObject,'Value')} returns selected item from drbRange


% --- Executes during object creation, after setting all properties.
function drbRange_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drbRange (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnSetRange.
function btnSetRange_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetRange (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sPort;
global bPortOpened;
if bPortOpened == 0
    return;
end
range = get(handles.drbRange, 'Value');

fwrite(sPort, 254, 'uint8');
fwrite(sPort, 255, 'uint8');
fwrite(sPort, 16+2, 'uint8'); %CMD
fwrite(sPort, range-1, 'uint8'); %CMD

byte1= fread(sPort, 1, 'uint8');
byte2 = fread(sPort, 1, 'uint8');
byte3= fread(sPort, 1, 'uint8');
fprintf("%d - %d - %d \n", byte1, byte2, byte3);

% --- Executes on selection change in drbGain.
function drbGain_Callback(hObject, eventdata, handles)
% hObject    handle to drbGain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns drbGain contents as cell array
%        contents{get(hObject,'Value')} returns selected item from drbGain


% --- Executes during object creation, after setting all properties.
function drbGain_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drbGain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnSetGain.
function btnSetGain_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetGain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global sPort;
global bPortOpened;
if bPortOpened == 0
    return;
end

gain = get(handles.drbGain, 'Value');
if gain ~= 1
    gain = 0;
end

fwrite(sPort, 254, 'uint8');
fwrite(sPort, 255, 'uint8');
fwrite(sPort, 16+3, 'uint8'); %CMD
fwrite(sPort, gain, 'uint8'); %CMD

byte1= fread(sPort, 1, 'uint8');
byte2 = fread(sPort, 1, 'uint8');
byte3= fread(sPort, 1, 'uint8');
fprintf("%d - %d - %d \n", byte1, byte2, byte3);


% --- Executes on button press in btnSetFreq.
function btnSetFreq_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetFreq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sPort;
global bPortOpened;
if bPortOpened == 0
    return;
end
val = get(handles.txtStartFreqReg, 'String');
val = str2num(val);
fwrite(sPort, 254, 'uint8');
fwrite(sPort, 255, 'uint8');
fwrite(sPort, 16+5, 'uint8'); %CMD
fwrite(sPort, val, 'uint32');

byte1= fread(sPort, 1, 'uint8');
byte2 = fread(sPort, 1, 'uint8');
byte3= fread(sPort, 1, 'uint8');
fprintf("%d - %d - %d \n", byte1, byte2, byte3);


% --- Executes on button press in btnSetInc.
function btnSetInc_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetInc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sPort;
global bPortOpened;
if bPortOpened == 0
    return;
end
val = get(handles.txtIncReg, 'String');
val = str2num(val);
fwrite(sPort, 254, 'uint8');
fwrite(sPort, 255, 'uint8');
fwrite(sPort, 16+6, 'uint8'); %CMD
fwrite(sPort, val, 'uint32');

byte1= fread(sPort, 1, 'uint8');
byte2 = fread(sPort, 1, 'uint8');
byte3= fread(sPort, 1, 'uint8');
fprintf("%d - %d - %d \n", byte1, byte2, byte3);


% --- Executes on button press in btnSetNumSamples.
function btnSetNumSamples_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetNumSamples (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sPort;
global bPortOpened;
if bPortOpened == 0
    return;
end
val = get(handles.txtNumSamplesReg, 'String');
val = str2num(val);
fwrite(sPort, 254, 'uint8');
fwrite(sPort, 255, 'uint8');
fwrite(sPort, 16+7, 'uint8'); %CMD
fwrite(sPort, val, 'uint16');

byte1= fread(sPort, 1, 'uint8');
byte2 = fread(sPort, 1, 'uint8');
byte3= fread(sPort, 1, 'uint8');
fprintf("%d - %d - %d \n", byte1, byte2, byte3);


% --- Executes on button press in btnSetSettling.
function btnSetSettling_Callback(hObject, eventdata, handles)
% hObject    handle to btnSetSettling (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btnReset.
function btnReset_Callback(hObject, eventdata, handles)
% hObject    handle to btnReset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sPort;
global bPortOpened;
if bPortOpened == 0
    return;
end

fwrite(sPort, 254, 'uint8');
fwrite(sPort, 255, 'uint8');
fwrite(sPort, 16+4, 'uint8'); %CMD

byte1= fread(sPort, 1, 'uint8');
byte2 = fread(sPort, 1, 'uint8');
byte3= fread(sPort, 1, 'uint8');
fprintf("%d - %d - %d \n", byte1, byte2, byte3);



function txtCalibrateRes_Callback(hObject, eventdata, handles)
% hObject    handle to txtCalibrateRes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtCalibrateRes as text
%        str2double(get(hObject,'String')) returns contents of txtCalibrateRes as a double


% --- Executes during object creation, after setting all properties.
function txtCalibrateRes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtCalibrateRes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnCalibrateRes.
function btnCalibrateRes_Callback(hObject, eventdata, handles)
% hObject    handle to btnCalibrateRes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global bPortOpened;
global sPort;
global gainFactor;
if (bPortOpened == 1)    
    try
        resVal = cast(str2num(get(handles.txtCalibrateRes, 'String')), 'uint32');
        disp(resVal);
        fwrite(sPort, 254, 'uint8');
        fwrite(sPort, 255, 'uint8');
        fwrite(sPort, 4, 'uint8'); %CMD
        fwrite(sPort, resVal, 'uint32');
        
        %Check header
        byte1 = fread(sPort, 1, 'uint8');
        if byte1 ~= 254 
            return;
        end
        byte2 = fread(sPort, 1, 'uint8');
        if byte2 ~= 255 
            return;
        end
        cmd = fread(sPort, 1, 'uint8');
        if cmd ~= 4 
            return;
        end
        r = fread(sPort, 1, 'int16');  %8
        i = fread(sPort, 1, 'int16');  %8
        mg = sqrt(r*r + i*i);
        gainFactor = 1.0 / (1e5*mg);
        fprintf("r = %d, i= %d, mag = %f, gain = %f \n", r, i, mg, gainFactor);
        disp(gainFactor);
        set(handles.txtGainFactor, 'String', num2str(gainFactor));
    catch ex        
        msgbox(ex.message);        
    end
end


function txtGainFactor_Callback(hObject, eventdata, handles)
% hObject    handle to txtGainFactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtGainFactor as text
%        str2double(get(hObject,'String')) returns contents of txtGainFactor as a double


% --- Executes during object creation, after setting all properties.
function txtGainFactor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtGainFactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnImpedance.
function btnImpedance_Callback(hObject, eventdata, handles)
% hObject    handle to btnImpedance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global bPortOpened;
global sPort;
global gainFactor;
if (bPortOpened == 1)    
    try       
        fwrite(sPort, 254, 'uint8');
        fwrite(sPort, 255, 'uint8');
        fwrite(sPort, 6, 'uint8'); %CMD        
        
        %Check header
        byte1 = fread(sPort, 1, 'uint8');
        if byte1 ~= 254 
            return;
        end
        byte2 = fread(sPort, 1, 'uint8');
        if byte2 ~= 255 
            return;
        end
        cmd = fread(sPort, 1, 'uint8');
        if cmd ~= 4 
            return;
        end
        r = fread(sPort, 1, 'int16');  %8
        i = fread(sPort, 1, 'int16');  %8
        mg = sqrt(r*r + i*i);
        impedance = 1/(mg*gainFactor);
        fprintf("r = %d, i= %d, mag = %f, imp = %f \n", r, i, mg, impedance);
        disp(impedance);
        set(handles.txtImpedance, 'String', num2str(impedance));
    catch ex        
        msgbox(ex.message);        
    end
end



function txtImpedance_Callback(hObject, eventdata, handles)
% hObject    handle to txtImpedance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtImpedance as text
%        str2double(get(hObject,'String')) returns contents of txtImpedance as a double


% --- Executes during object creation, after setting all properties.
function txtImpedance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtImpedance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in chkLoRa.
function chkLoRa_Callback(hObject, eventdata, handles)
% hObject    handle to chkLoRa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkLoRa
