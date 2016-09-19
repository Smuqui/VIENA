function varargout = currentModeGUI(varargin)
% CURRENTMODEGUI MATLAB code for currentModeGUI.fig
%      CURRENTMODEGUI, by itself, creates a new CURRENTMODEGUI or raises the existing
%      singleton*.
%
%      H = CURRENTMODEGUI returns the handle to a new CURRENTMODEGUI or the handle to
%      the existing singleton*.
%
%      CURRENTMODEGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CURRENTMODEGUI.M with the given input arguments.
%
%      CURRENTMODEGUI('Property','Value',...) creates a new CURRENTMODEGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before currentModeGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to currentModeGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help currentModeGUI

% Last Modified by GUIDE v2.5 19-Sep-2016 13:25:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @currentModeGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @currentModeGUI_OutputFcn, ...
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


% --- Executes just before currentModeGUI is made visible.
function currentModeGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to currentModeGUI (see VARARGIN)

% Choose default command line output for currentModeGUI
handles.output = hObject;
addpath('../');
epos = Epos();
assignin('base','epos',epos);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes currentModeGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = currentModeGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in activate_pushbutton.
function activate_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to activate_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function accelaration_slider_Callback(hObject, eventdata, handles)
% hObject    handle to accelaration_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function accelaration_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to accelaration_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function ki_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ki_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ki_edit as text
%        str2double(get(hObject,'String')) returns contents of ki_edit as a double


% --- Executes during object creation, after setting all properties.
function ki_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ki_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function kp_edit_Callback(hObject, eventdata, handles)
% hObject    handle to kp_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of kp_edit as text
%        str2double(get(hObject,'String')) returns contents of kp_edit as a double


% --- Executes during object creation, after setting all properties.
function kp_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kp_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in reset_pushbutton.
function reset_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to reset_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
epos = evalin('base','epos');
OK = epos.changeEposState('fault reset');



function current_edit_Callback(hObject, eventdata, handles)
% hObject    handle to current_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of current_edit as text
%        str2double(get(hObject,'String')) returns contents of current_edit as a double


% --- Executes during object creation, after setting all properties.
function current_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function speed_edit_Callback(hObject, eventdata, handles)
% hObject    handle to speed_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of speed_edit as text
%        str2double(get(hObject,'String')) returns contents of speed_edit as a double


% --- Executes during object creation, after setting all properties.
function speed_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to speed_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pole_pair_edit_Callback(hObject, eventdata, handles)
% hObject    handle to pole_pair_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pole_pair_edit as text
%        str2double(get(hObject,'String')) returns contents of pole_pair_edit as a double


% --- Executes during object creation, after setting all properties.
function pole_pair_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pole_pair_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor_type_edit_Callback(hObject, eventdata, handles)
% hObject    handle to motor_type_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor_type_edit as text
%        str2double(get(hObject,'String')) returns contents of motor_type_edit as a double


% --- Executes during object creation, after setting all properties.
function motor_type_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor_type_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function port_edit_Callback(hObject, eventdata, handles)
% hObject    handle to port_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of port_edit as text
%        str2double(get(hObject,'String')) returns contents of port_edit as a double


% --- Executes during object creation, after setting all properties.
function port_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to port_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in motor_type_popuomenu.
function motor_type_popuomenu_Callback(hObject, eventdata, handles)
% hObject    handle to motor_type_popuomenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns motor_type_popuomenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from motor_type_popuomenu


% --- Executes during object creation, after setting all properties.
function motor_type_popuomenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor_type_popuomenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in connect_pushbutton.
function connect_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to connect_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
epos = evalin('base','epos');
port_string = get(handles.port_edit, 'String');
if epos.connected == false
	OK = epos.begin(port_string);
	if OK
		set(handles.port_edit, 'Enable', 'inactive');
		set(handles.connect_pushbutton, 'String','Disconnect');
	end
else
	epos.disconnect();
	set(handles.port_edit, 'Enable','on');
	set(handles.connect_pushbutton, 'String','Connect');
end




% --- Executes on button press in apply_motorspecs_pushbutton.
function apply_motorspecs_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to apply_motorspecs_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in apply_gain_pushbutton.
function apply_gain_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to apply_gain_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
epos = evalin('base','epos');
epos.disconnect();
delete(hObject);
