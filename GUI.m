
function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 26-Jun-2015 00:59:40

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
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


% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)

% Choose default command line output for GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function C1_Callback(hObject, eventdata, handles)
% hObject    handle to C1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of C1 as text
%        str2double(get(hObject,'String')) returns contents of C1 as a double


% --- Executes during object creation, after setting all properties.
function C1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to C1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function h_Callback(hObject, eventdata, handles)
% hObject    handle to h (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of h as text
%        str2double(get(hObject,'String')) returns contents of h as a double


% --- Executes during object creation, after setting all properties.
function h_CreateFcn(hObject, eventdata, handles)
% hObject    handle to h (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function j_Callback(hObject, eventdata, handles)
% hObject    handle to j (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of j as text
%        str2double(get(hObject,'String')) returns contents of j as a double


% --- Executes during object creation, after setting all properties.
function j_CreateFcn(hObject, eventdata, handles)
% hObject    handle to j (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

C11=str2num(get(handles.C1,'String'));
C12=str2num(get(handles.C2,'String'));
C13=str2num(get(handles.C3,'String'));
h=str2num(get(handles.h,'String'));
j=str2num(get(handles.j,'String'));

%% Given system
R = 1;
M = 1;
k = 1;
L = 0.01;
g = 9.8;
yo = 0.5;

syms x1 x2 x3
syms xdot1 xdot2 xdot3
syms v y

xdot1 = x2;
xdot2 = g - (k/M)*(x3^2/x1);
xdot3 = -(R/L) * x3 + v/L;

a = jacobian([xdot1; xdot2; xdot3],[x1, x2, x3]);
b = jacobian([xdot1; xdot2; xdot3],v);

%initials
x1o = yo;
x2o = 0;
x3o = sqrt((M*g/k)*(x1o));
vo = R*x3o;


A = double(subs(a,[x1, x2, x3, v],[x1o, x2o, x3o, vo]));
B = double(subs(b,[x1, x2, x3, v],[x1o, x2o, x3o, vo]));
C = [C11 C12 C13];
D = 0;


t = 0:0.01:0.8;
u = zeros(size(t));
x0 = [0.05 0.05 0.05]; % the ball is initially at 0.05m


%% controller
p1 = complex(h,-j);
p2 = complex(h, j);
K = place(A,B,[-20 p1 p2]);
CONT = ss(A-B*K,B,C,D);
[output_cont,t,x_cont] = lsim(CONT,u,t,x0);
CONT_prop = lsiminfo(x_cont(:,1),t,0);

%% total system
op1 = -100;
op2 = -101;
op3 = -102;

L = place(A',C',[op1 op2 op3])';

Atot = [A-B*K B*K; zeros(size(A)) A-L*C];
Btot = [B; zeros(size(B))];
Ctot = [C zeros(size(C))];
Dtot = 0;

e0 = [-0.1 -0.1 -0.1]; % assumed all intinal errors are -0.1
TOT_SYS = ss(Atot,Btot,Ctot,Dtot);
[output_tot,t,x_tot] = lsim(TOT_SYS,u,t,[x0 e0]);
TOT_SYS_prop = lsiminfo(x_tot(:,1),t,0);
axes(handles.axes1);
plot(t,x_tot(:,1),'black')
hold on
plot(t,x_cont(:,1),'red')
title('Total System Response Vs State-Feedback System Response')
legend('with Observer','Feedback only')
grid on

out_time = num2str(TOT_SYS_prop.SettlingTime);
 set(handles.text9,'string',out_time);
 
 out_dev= num2str(TOT_SYS_prop.Max);
 set(handles.text11,'string',out_dev);

function C2_Callback(hObject, eventdata, handles)
% hObject    handle to C2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of C2 as text
%        str2double(get(hObject,'String')) returns contents of C2 as a double


% --- Executes during object creation, after setting all properties.
function C2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to C2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function C3_Callback(hObject, eventdata, handles)
% hObject    handle to C3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of C3 as text
%        str2double(get(hObject,'String')) returns contents of C3 as a double


% --- Executes during object creation, after setting all properties.
function C3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to C3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
