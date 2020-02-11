function varargout = matlab_gui(varargin)
% MATLAB_GUI MATLAB code for matlab_gui.fig
%      MATLAB_GUI, by itself, creates a new MATLAB_GUI or raises the existing
%      singleton*.
%
%      H = MATLAB_GUI returns the handle to a new MATLAB_GUI or the handle to
%      the existing singleton*.
%
%      MATLAB_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MATLAB_GUI.M with the given input arguments.
%
%      MATLAB_GUI('Property','Value',...) creates a new MATLAB_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before matlab_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to matlab_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help matlab_gui

% Last Modified by GUIDE v2.5 10-Feb-2020 19:33:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @matlab_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @matlab_gui_OutputFcn, ...
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


% --- Executes just before matlab_gui is made visible.
function matlab_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to matlab_gui (see VARARGIN)

% Choose default command line output for matlab_gui
handles.output = hObject;

% Create a timer object to fire at 1/10 sec intervals
% Specify function handles for its start and run callbacks
handles.timer = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 1, ...                        % Initial period is 1 sec.
    'TimerFcn', {@update_display,hObject}); % Specify callback function

handles.serial = serial("COM7", "BaudRate", 9600);
fopen(handles.serial);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes matlab_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = matlab_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% START USER CODE
function update_display(hObject,eventdata,hfigure)
% Timer timer1 callback, called each time timer iterates.
% Gets surface Z data, adds noise, and writes it back to surface object.

handles = guidata(hfigure);
set(handles.text5, "String", fscanf(handles.serial));
% END USER CODE