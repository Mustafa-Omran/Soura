function varargout = wrt(varargin)
% WRT MATLAB code for wrt.fig
%      WRT, by itself, creates a new WRT or raises the existing
%      singleton*.
%
%      H = WRT returns the handle to a new WRT or the handle to
%      the existing singleton*.
%
%      WRT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in WRT.M with the given input arguments.
%
%      WRT('Property','Value',...) creates a new WRT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before wrt_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to wrt_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help wrt

% Last Modified by GUIDE v2.5 20-Jan-2016 12:42:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @wrt_OpeningFcn, ...
                   'gui_OutputFcn',  @wrt_OutputFcn, ...
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


% --- Executes just before wrt is made visible.
function wrt_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to wrt (see VARARGIN)

% Choose default command line output for wrt
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes wrt wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = wrt_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear all
clc
%Detect objects using Viola-Jones Algorithm
%To detect Face
FDetect = vision.CascadeObjectDetector;
%Read the input image
I = imread('test.jpg');
%Returns Bounding Box values based on number of objects
BB = step(FDetect,I);
imshow(I); hold on
for i = 1:size(BB,1)
rectangle('Position',BB(i,:),'LineWidth',2,'LineStyle','-','EdgeColor','r');
end
title('Face Detection');
hold off;


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
NoseDetect = vision.CascadeObjectDetector('Nose','MergeThreshold',16);
I = imread('test.jpg');
BB=step(NoseDetect,I);
imshow(I); hold on
for i = 1:size(BB,1)
rectangle('Position',BB(i,:),'LineWidth',2,'LineStyle','-','EdgeColor','b');
end
title('Nose Detection');
hold off;


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%To detect Mouth
MouthDetect = vision.CascadeObjectDetector('Mouth','MergeThreshold',16);
I = imread('test.jpg');
BB=step(MouthDetect,I);
imshow(I); hold on
for i = 1:size(BB,1)
rectangle('Position',BB(i,:),'LineWidth',2,'LineStyle','-','EdgeColor','g');
end
title('Mouth Detection');
hold off;


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=imread('test.jpg');
a=rgb2gray(x);
[r c]=size(a);
s=3;
ng=zeros(r*s ,c*s ,class(a));
for i=1:r*s
for j=1 :c*s
ng(i,j) = a(ceil(i/s),ceil(j/s));
end
end
imshow(ng);
title('Grayscale Zooming');

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
x=imread('test.jpg');
g=rgb2gray(x);
[r c]=size(g);
cg = zeros(r,c,3,class(g));
for i=1:r
for j=1:c
cg(i,j,1)=g(i,j);
cg(i,j,2)=g(i,j);
cg(i,j,3)=g(i,j);
end
end
ng=g>100;
for i=1:r
for j=1:c
if ng(i,j)==0
cg(i,j,1)=255;
cg(i,j,2)=0;
cg(i,j,3)=0;
end
end
end
imshow(ng);
title('Thresholding');

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% Rotation Handle From 1 to end and rotate them
RGB = imread('test.jpg');
LR = RGB(1:end,end:-1:1,:);
imshow(LR);
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
RGB = imread('test.jpg');
LR = RGB(end:-1:1,1:end,:);
imshow(LR);


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
RGB = imread('test.jpg');
LR = RGB(end:-1:1,1:end,:);
imshow(LR);


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
RGB = imread('test.jpg');
imshow(RGB);


% --- Executes during object creation, after setting all properties.
function bg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate bg
axes(handles.bg);
imshow('test.jpg');


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Q=imread('test.jpg');
for r=1:size(Q,1);
for c=1:size(Q,2);
Q(r,c,1)=Q(r,c,1)+50;
Q(r,c,2)=Q(r,c,2)+50;
Q(r,c,3)=Q(r,c,3)+50;
end
end
imshow(Q);
% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Q=imread('test.jpg');
for r=1:size(Q,1);
for c=1:size(Q,2);
Q(r,c,1)=Q(r,c,1)-50;
Q(r,c,2)=Q(r,c,2)-50;
Q(r,c,3)=Q(r,c,3)-50;
end
end
imshow(Q);


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
x=imread('test.jpg');
imshow(x);


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close all;


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=imread('test.jpg');
for i=1:size(x,1)
    for j=1:size(x,2)
      Result(i,j)=(x(i,j,1)*.3)+(x(i,j,2)*.59)+(x(i,j,3)*.11);
    end
end
imshow(Result);
title('Grayscale');
% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
i = imread('test.jpg');
colcounter = 1;
rowcounter = 1;
for r=1 : size(i,1)
    for c=1:size(i,2);
        Rzoomed(rowcounter:rowcounter+1,colcounter:colcounter+1)=i(r,c,1);
        Gzoomed(rowcounter:rowcounter+1,colcounter:colcounter+1)=i(r,c,2);
        Bzoomed(rowcounter:rowcounter+1,colcounter:colcounter+1)=i(r,c,3);
        
        colcounter = colcounter+2;
    end
    rowcounter = rowcounter+2;
    colcounter = 1;
end
image = cat(3,Rzoomed,Gzoomed,Bzoomed);
imshow(image);


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Q=imread('test.jpg');
for r=1:size(Q,1);
for c=1:size(Q,2);
Q(r,c,1)=255-Q(r,c,1);
Q(r,c,2)=255-Q(r,c,2);
Q(r,c,3)=255-Q(r,c,3);
end
end
imshow(Q);


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Q=imread('test.jpg');
for r=1:size(Q,1);
for c=1:size(Q,2);
Q(r,c,1)=Q(r,c,1)*2;
Q(r,c,2)=Q(r,c,2)*2;
Q(r,c,3)=Q(r,c,3)*2;
end
end
imshow(Q);
