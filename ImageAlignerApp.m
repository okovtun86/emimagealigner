classdef ImageAlignerApp < matlab.apps.AppBase
    properties (Access = public)
        UIFigure            matlab.ui.Figure
        Grid                matlab.ui.container.GridLayout
        LeftPanel           matlab.ui.container.Panel
        RightPanel          matlab.ui.container.Panel
        ControlsPanel       matlab.ui.container.Panel
        FixedAxes           matlab.ui.control.UIAxes
        MovingAxes          matlab.ui.control.UIAxes
        OverlayAxes         matlab.ui.control.UIAxes
        LoadFixedBtn        matlab.ui.control.Button
        LoadMovingBtn       matlab.ui.control.Button
        ManualPtsBtn        matlab.ui.control.Button
        ApplyBtn            matlab.ui.control.Button
        ResetBtn            matlab.ui.control.Button
        SaveBtn             matlab.ui.control.Button
        ModelDrop           matlab.ui.control.DropDown
        ModelLabel          matlab.ui.control.Label
        UseRansacChk        matlab.ui.control.CheckBox
        TolLabel            matlab.ui.control.Label
        TolEdit             matlab.ui.control.NumericEditField
        AlphaSlider         matlab.ui.control.Slider
        AlphaLabel          matlab.ui.control.Label
        StatusLbl           matlab.ui.control.Label
        NudgeEnable         matlab.ui.control.Button
        NudgeCommit         matlab.ui.control.Button
        NudgeCancel         matlab.ui.control.Button
    end
    properties (Access = private)
        I_fixed
        I_moving
        I_warped
        tformInit
        tformChain
        cpFixed
        cpMoving
        availableModels cell
        modernGeo logical
        pendingTform
        nudgeActive logical
        previewScale double
    end
    methods (Access = public)
        function app = ImageAlignerApp
            app.modernGeo = exist('fitgeotform2d','file')==2;
            app.availableModels = app.buildModelList();
            app.previewScale = 1; 
            app.nudgeActive = false;
            buildUI(app);
            app.updateStatus('Ready. Load fixed and moving images.');
        end
    end
    methods (Access = private)
        function mdl = buildModelList(app)
            mdl = {'translation','rigid','similarity','affine','projective'};
            if app.modernGeo
                mdl = [mdl, {'polynomial2','pwl','lwm'}];
            else
                mdl = [mdl, {'pwl'}];
            end
        end
        function buildUI(app)
            app.UIFigure = uifigure('Name','EM Aligner (Nudge)','Position',[60 60 1200 820]);
            app.UIFigure.KeyPressFcn = @(src,evt)onKeyAdjust(app,evt);
            app.Grid = uigridlayout(app.UIFigure,[2 2]);
            app.Grid.RowHeight = {'1x', 120};
            app.Grid.ColumnWidth = {'1x','1x'};
            app.LeftPanel = uipanel(app.Grid,'Title','Inputs (Fixed / Moving)');
            app.LeftPanel.Layout.Row = 1; app.LeftPanel.Layout.Column = 1;
            leftGrid = uigridlayout(app.LeftPanel,[2 1]); leftGrid.RowHeight = {'1x','1x'};
            app.FixedAxes = uiaxes(leftGrid); title(app.FixedAxes,'Fixed (FIB-SEM)'); axis(app.FixedAxes,'image');
            app.MovingAxes = uiaxes(leftGrid); title(app.MovingAxes,'Moving (SEM)'); axis(app.MovingAxes,'image');
            app.RightPanel = uipanel(app.Grid,'Title','Overlay');
            app.RightPanel.Layout.Row = 1; app.RightPanel.Layout.Column = 2;
            app.OverlayAxes = uiaxes(app.RightPanel,'Position',[20 20 640 640]);
            title(app.OverlayAxes,'Overlay'); axis(app.OverlayAxes,'image');
            app.ControlsPanel = uipanel(app.Grid,'Title','Controls');
            app.ControlsPanel.Layout.Row = 2; app.ControlsPanel.Layout.Column = [1 2];
            ctrl = uigridlayout(app.ControlsPanel,[2 10]);
            ctrl.RowHeight = {34,34};
            ctrl.ColumnWidth = {100,100,110,120,120,110,110,120,120,'1x'};
            app.LoadFixedBtn = uibutton(ctrl,'push','Text','Load Fixed','ButtonPushedFcn',@(~,~)onLoadFixed(app)); app.LoadFixedBtn.Layout.Row=1; app.LoadFixedBtn.Layout.Column=1;
            app.LoadMovingBtn = uibutton(ctrl,'push','Text','Load Moving','ButtonPushedFcn',@(~,~)onLoadMoving(app)); app.LoadMovingBtn.Layout.Row=1; app.LoadMovingBtn.Layout.Column=2;
            app.ModelLabel = mkLabel(ctrl,'Model:'); app.ModelLabel.Layout.Row=1; app.ModelLabel.Layout.Column=3;
            app.ModelDrop  = uidropdown(ctrl,'Items',app.availableModels,'Value',app.availableModels{4}); app.ModelDrop.Layout.Row=1; app.ModelDrop.Layout.Column=4;
            app.ManualPtsBtn = uibutton(ctrl,'push','Text','Pick Landmarks','ButtonPushedFcn',@(~,~)onManualPoints(app)); app.ManualPtsBtn.Layout.Row=1; app.ManualPtsBtn.Layout.Column=5;
            app.ApplyBtn  = uibutton(ctrl,'push','Text','Apply Transform','ButtonPushedFcn',@(~,~)onApply(app)); app.ApplyBtn.Layout.Row=1; app.ApplyBtn.Layout.Column=6;
            app.ResetBtn = uibutton(ctrl,'push','Text','Reset','ButtonPushedFcn',@(~,~)onReset(app)); app.ResetBtn.Layout.Row=1; app.ResetBtn.Layout.Column=7;
            app.SaveBtn  = uibutton(ctrl,'push','Text','Save Result','ButtonPushedFcn',@(~,~)onSave(app)); app.SaveBtn.Layout.Row=1; app.SaveBtn.Layout.Column=8;
            app.AlphaLabel = mkLabel(ctrl,'Overlay alpha'); app.AlphaLabel.Layout.Row=1; app.AlphaLabel.Layout.Column=9;
            app.AlphaSlider = uislider(ctrl,'Limits',[0 1],'Value',0.5,'ValueChangedFcn',@(~,~)updateOverlay(app)); app.AlphaSlider.Layout.Row=1; app.AlphaSlider.Layout.Column=10;
            app.StatusLbl = mkLabel(ctrl,'Status...'); app.StatusLbl.Layout.Row=1; app.StatusLbl.Layout.Column=11;
            app.UseRansacChk = uicheckbox(ctrl,'Text','RANSAC','Value',true); app.UseRansacChk.Layout.Row=2; app.UseRansacChk.Layout.Column=1;
            app.TolLabel = mkLabel(ctrl,'Tol (px):'); app.TolLabel.Layout.Row=2; app.TolLabel.Layout.Column=2;
            app.TolEdit = uieditfield(ctrl,'numeric','Value',3,'Limits',[0 Inf]); app.TolEdit.Layout.Row=2; app.TolEdit.Layout.Column=3;
            app.NudgeEnable = uibutton(ctrl,'push','Text','Nudge: Enable','ButtonPushedFcn',@(~,~)onNudgeEnable(app)); app.NudgeEnable.Layout.Row=2; app.NudgeEnable.Layout.Column=4;
            app.NudgeCommit = uibutton(ctrl,'push','Text','Nudge: Commit','ButtonPushedFcn',@(~,~)onNudgeCommit(app)); app.NudgeCommit.Layout.Row=2; app.NudgeCommit.Layout.Column=5;
            app.NudgeCancel = uibutton(ctrl,'push','Text','Nudge: Cancel','ButtonPushedFcn',@(~,~)onNudgeCancel(app)); app.NudgeCancel.Layout.Row=2; app.NudgeCancel.Layout.Column=6;
        end
        function onLoadFixed(app)
            [f,p] = uigetfile({'*.*','All files';'*.tif;*.png;*.jpg','Images'});
            if isequal(f,0), return; end
            app.I_fixed = ensureRGBorGray(imread(fullfile(p,f)));
            imshow(app.I_fixed,'Parent',app.FixedAxes);
            app.updateStatus('Loaded fixed');
            updateOverlay(app);
            lockAxesToFixedRef(app);
        end
        function onLoadMoving(app)
            [f,p] = uigetfile({'*.*','All files';'*.tif;*.png;*.jpg','Images'});
            if isequal(f,0), return; end
            app.I_moving = ensureRGBorGray(imread(fullfile(p,f)));
            app.I_warped = [];
            imshow(app.I_moving,'Parent',app.MovingAxes);
            app.updateStatus('Loaded moving');
            updateOverlay(app);
            lockAxesToFixedRef(app);
        end
        function onManualPoints(app)
            if isempty(app.I_fixed) || isempty(app.I_moving)
                app.updateStatus('Load both first'); return; end
            modelUI = app.ModelDrop.Value; [modelCore, polyOrder, lwmK] = app.parseModel(modelUI);
            nMin = app.minReqPoints(modelCore, polyOrder);
            app.updateStatus(sprintf('Pick ≥%d points, then Accept',nMin));
            try
                [mp, fp] = cpselect(app.I_moving, app.I_fixed, 'Wait', true);
            catch ME
                app.updateStatus(['cpselect: ' ME.message]); return;
            end
            if size(mp,1) < nMin
                app.updateStatus('Insufficient points'); return;
            end
            app.cpMoving = mp; app.cpFixed = fp;
            try
                robust = app.UseRansacChk.Value; tol = app.TolEdit.Value;
                app.tformInit = app.estimateTform(mp, fp, modelCore, polyOrder, lwmK, robust, tol);
                app.updateStatus(['Estimated ' modelUI]);
                onApply(app);
            catch ME
                app.updateStatus(['Transform: ' ME.message]);
            end
        end
        function onApply(app)
            if isempty(app.I_fixed) || isempty(app.I_moving) || isempty(app.tformInit)
                app.updateStatus('Nothing to apply'); return; end
            app.tformChain = composeTransforms(app.tformChain, app.tformInit);
            app.tformInit = [];
            RA = imref2d(size(toGray(app.I_fixed)));
            app.I_warped = imwarp(app.I_moving, app.tformChain,'OutputView',RA);
            imshow(app.I_warped,'Parent',app.MovingAxes);
            updateOverlay(app);
            lockAxesToFixedRef(app);
            app.updateStatus('Applied');
        end
        function onNudgeEnable(app)
            if isempty(app.I_fixed) || isempty(app.I_moving)
                app.updateStatus('Load images first'); return; end
            app.nudgeActive = true;
            app.pendingTform = projective2d(eye(3)); 
            app.updateStatus('Nudge: arrows move, </> rotate, 9/0 or -/= scaleX, ;/'' scaleY, 1-4 shear, Enter=commit, Esc=cancel');
            lockAxesToFixedRef(app);
        end
        function onNudgeCommit(app)
            if ~app.nudgeActive || isempty(app.pendingTform)
                app.updateStatus('Nothing to commit'); return; end
            app.tformChain = composeTransforms(app.tformChain, app.pendingTform);
            app.pendingTform = [];
            app.nudgeActive = false;
            RA = imref2d(size(toGray(app.I_fixed)));
            app.I_warped = imwarp(app.I_moving, app.tformChain,'OutputView',RA);
            imshow(app.I_warped,'Parent',app.MovingAxes);
            updateOverlay(app);
            lockAxesToFixedRef(app);
            app.updateStatus('Committed');
        end
        function onNudgeCancel(app)
            app.pendingTform = [];
            app.nudgeActive = false;
            if ~isempty(app.I_moving) && ~isempty(app.I_fixed)
                RA = imref2d(size(toGray(app.I_fixed)));
                if isempty(app.tformChain)
                    app.I_warped = app.I_moving;
                else
                    app.I_warped = imwarp(app.I_moving, app.tformChain,'OutputView',RA);
                end
                imshow(app.I_warped,'Parent',app.MovingAxes);
                updateOverlay(app);
                lockAxesToFixedRef(app);
            end
            app.updateStatus('Cancelled');
        end
        function onKeyAdjust(app, evt)
            if ~app.nudgeActive || isempty(app.I_fixed) || isempty(app.I_moving), return; end
            if isempty(app.pendingTform), app.pendingTform = projective2d(eye(3)); end
            big = false; if isfield(evt,'Modifier') && ~isempty(evt.Modifier), big = any(strcmpi(evt.Modifier,'shift')); end
            stepT = eye(3);
            baseRot = pi/720; baseScale = 1.005; baseShear = pi/360;
            if strcmp(evt.Key,'rightarrow')
                dx = big * 10 + (~big) * 1; stepT(3,1)=dx;
            elseif strcmp(evt.Key,'leftarrow')
                dx = big * 10 + (~big) * 1; stepT(3,1)=-dx;
            elseif strcmp(evt.Key,'uparrow')
                dy = big * 10 + (~big) * 1; stepT(3,2)=-dy;
            elseif strcmp(evt.Key,'downarrow')
                dy = big * 10 + (~big) * 1; stepT(3,2)=dy;
            elseif strcmp(evt.Key,'comma')
                k = big*4 + (~big)*1; th = -k*baseRot; stepT = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
            elseif strcmp(evt.Key,'period')
                k = big*4 + (~big)*1; th =  k*baseRot; stepT = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
            elseif strcmp(evt.Key,'9') || (isfield(evt,'Character') && strcmp(evt.Character,'9')) ...
                   || strcmpi(evt.Key,'subtract') || strcmpi(evt.Key,'minus') || strcmpi(evt.Key,'hyphen') || (isfield(evt,'Character') && strcmp(evt.Character,'-'))
                k = big*4 + (~big)*1; s = baseScale^(-k); stepT = [s 0 0; 0 1 0; 0 0 1];
            elseif strcmp(evt.Key,'0') || (isfield(evt,'Character') && strcmp(evt.Character,'0')) ...
                   || strcmpi(evt.Key,'equal') || (isfield(evt,'Character') && strcmp(evt.Character,'='))
                k = big*4 + (~big)*1; s = baseScale^( k); stepT = [s 0 0; 0 1 0; 0 0 1];
            elseif strcmp(evt.Key,'semicolon') || (isfield(evt,'Character') && strcmp(evt.Character,';'))
                k = big*4 + (~big)*1; s = baseScale^(-k); stepT = [1 0 0; 0 s 0; 0 0 1];
            elseif strcmp(evt.Key,'quote') || strcmpi(evt.Key,'apostrophe') || (isfield(evt,'Character') && strcmp(evt.Character,'''"(?)'))
                k = big*4 + (~big)*1; s = baseScale^( k); stepT = [1 0 0; 0 s 0; 0 0 1];
            elseif strcmp(evt.Key,'1')
                k = big*4 + (~big)*1; sh = -k*baseShear; stepT = [1 tan(sh) 0; 0 1 0; 0 0 1];
            elseif strcmp(evt.Key,'2')
                k = big*4 + (~big)*1; sh =  k*baseShear; stepT = [1 tan(sh) 0; 0 1 0; 0 0 1];
            elseif strcmp(evt.Key,'3')
                k = big*4 + (~big)*1; sh = -k*baseShear; stepT = [1 0 0; tan(sh) 1 0; 0 0 1];
            elseif strcmp(evt.Key,'4')
                k = big*4 + (~big)*1; sh =  k*baseShear; stepT = [1 0 0; tan(sh) 1 0; 0 0 1];
            elseif strcmp(evt.Key,'return')
                onNudgeCommit(app); return;
            elseif strcmp(evt.Key,'escape')
                onNudgeCancel(app); return;
            else
                return;
            end
            A_step    = stepT;
            A_pending = getMatrix(app.pendingTform);
            A_new     = A_step * A_pending;
            app.pendingTform = makeTformFromMatrix(A_new);
            RA = imref2d(size(toGray(app.I_fixed)));
            if isempty(app.tformChain)
                Tw = app.pendingTform;
            else
                A_chain = getMatrix(app.tformChain);
                Tw = makeTformFromMatrix(app.pendingTform.T * A_chain);
            end
            Iin = app.I_moving; s = app.previewScale; if s<1, Iin = imresize(Iin, s); end
            try
                Iprev = imwarp(im2single(Iin), Tw, 'OutputView', RA, 'InterpolationMethod','nearest');
            catch
                Iprev = imwarp(Iin, Tw, 'OutputView', RA, 'InterpolationMethod','nearest');
            end
            imshow(Iprev,'Parent',app.MovingAxes);
            app.I_warped = Iprev;
            updateOverlay(app);
            lockAxesToFixedRef(app);
        end
        function onReset(app)
            app.tformInit = []; app.tformChain = [];
            app.I_warped = []; app.cpFixed = []; app.cpMoving = [];
            app.pendingTform = [];
            app.nudgeActive = false;
            if ~isempty(app.I_moving)
                imshow(app.I_moving,'Parent',app.MovingAxes);
            end
            updateOverlay(app);
            lockAxesToFixedRef(app);
            app.updateStatus('Reset');
        end
        function onSave(app)
            if isempty(app.I_warped)
                app.updateStatus('Nothing to save'); return; end
            [f,p] = uiputfile({'*.tif','TIFF';'*.png','PNG';'*.jpg','JPEG'});
            if isequal(f,0), return; end
            imwrite(normalizeForSave(app.I_warped), fullfile(p,f));
            app.updateStatus('Saved');
        end
        function updateOverlay(app)
            if isempty(app.I_fixed) || (isempty(app.I_warped) && isempty(app.I_moving))
                cla(app.OverlayAxes); return; end
            alpha = app.AlphaSlider.Value;
            If = im2single(app.I_fixed);
            if ~isempty(app.I_warped)
                Im = im2single(app.I_warped);
            else
                Im = im2single(app.I_moving);
            end
            [If2,Im2] = matchSizes(If,Im);
            If2 = toRGB(If2); Im2 = toRGB(Im2);
            rgb = alpha*Im2 + (1-alpha)*If2;
            imshow(rgb,'Parent',app.OverlayAxes);
            lockAxesToFixedRef(app);
        end
        function updateStatus(app,msg)
            try
                app.StatusLbl.Text = msg;
            catch
                fprintf('%s',msg);
            end
            drawnow;
        end
        function [modelCore, polyOrder, lwmK] = parseModel(~, modelUI)
            polyOrder = []; lwmK = [];
            switch lower(modelUI)
                case 'polynomial2'
                    modelCore = 'polynomial'; polyOrder = 2;
                case 'lwm'
                    modelCore = 'lwm'; lwmK = 12;
                otherwise
                    modelCore = lower(modelUI);
            end
        end
        function tform = estimateTform(app, mp, fp, modelCore, polyOrder, lwmK, robust, tol)
            canRobust = ismember(modelCore, {'translation','rigid','similarity','affine','projective','polynomial','pwl'});
            if robust && canRobust
                try
                    if strcmp(modelCore,'polynomial')
                        tform = estimateGeometricTransform2D(mp, fp, 'polynomial', 'MaxNumTrials', 5000, 'Confidence', 99.5, 'MaxDistance', tol, 'PolynomialDegree', polyOrder);
                        return;
                    else
                        tform = estimateGeometricTransform2D(mp, fp, modelCore, 'MaxNumTrials', 5000, 'Confidence', 99.5, 'MaxDistance', tol);
                        return;
                    end
                catch
                end
            end
            if app.modernGeo
                switch modelCore
                    case 'polynomial'
                        tform = fitgeotform2d(mp, fp, 'polynomial', 'Order', polyOrder);
                    case 'pwl'
                        tform = fitgeotform2d(mp, fp, 'pwl');
                    case 'lwm'
                        tform = fitgeotform2d(mp, fp, 'lwm', 'NumNeighbors', lwmK);
                    otherwise
                        tform = fitgeotform2d(mp, fp, modelCore);
                end
            else
                switch modelCore
                    case 'polynomial'
                        tform = fitgeotrans(mp, fp, 'polynomial', polyOrder);
                    case 'pwl'
                        tform = fitgeotrans(mp, fp, 'pwl');
                    case 'lwm'
                        tform = fitgeotrans(mp, fp, 'pwl');
                    otherwise
                        tform = fitgeotrans(mp, fp, modelCore);
                end
            end
        end
        function n = minReqPoints(app, modelCore, polyOrder)
            switch lower(modelCore)
                case 'translation', n = 1;
                case 'rigid',       n = 2;
                case 'similarity',  n = 2;
                case 'affine',      n = 3;
                case 'projective',  n = 4;
                case 'pwl',         n = 4;
                case 'polynomial'
                    if isempty(polyOrder), polyOrder = 2; end
                    terms = (polyOrder+1)*(polyOrder+2)/2; n = terms;
                case 'lwm'
                    n = 12;
                otherwise
                    n = 3;
            end
        end
    end
end
function I = ensureRGBorGray(I)
    if ndims(I)==2
    elseif ndims(I)==3 && size(I,3)==3
    else
        I = I(:,:,1);
    end
    if isfloat(I), I = min(max(I,0),1); end
end
function g = toGray(I)
    if size(I,3)==3, g = rgb2gray(im2single(I)); else, g = im2single(I); end
end
function R = normalizeForSave(I)
    if isinteger(I), R = I; else, R = mat2gray(I); end
end
function [A2,B2] = matchSizes(A,B)
    sA=size(A); sB=size(B);
    H=max(sA(1),sB(1)); W=max(sA(2),sB(2));
    A2=zeros([H W size(A,3)],'like',A); B2=zeros([H W size(B,3)],'like',B);
    A2(1:sA(1),1:sA(2),:)=A; B2(1:sB(1),1:sB(2),:)=B;
end
function R = toRGB(I)
    if size(I,3)==1, R = repmat(I,1,1,3); else, R = I; end
end
function A = getMatrix(T)
    if isa(T,'affine2d') || isa(T,'projective2d')
        A = T.T;
    else
        A = eye(3);
    end
end
function T = makeTformFromMatrix(A)
    T = projective2d(A);
end
function T = composeTransforms(Tprev,Tnew)
    if isempty(Tprev), T=Tnew; return; end
    Aprev=getMatrix(Tprev); Anew=getMatrix(Tnew);
    Acomp=Anew*Aprev;
    T=makeTformFromMatrix(Acomp);
end
function A=getAffine(T)
    if isa(T,'affine2d'), A=T; elseif isa(T,'projective2d'), A=affine2d(T.T); else, A=affine2d(eye(3)); end
end
function lbl = mkLabel(parent, txt)
    try
        lbl = uilabel(parent,'Text',txt);
    catch
        lbl = matlab.ui.control.Label(parent); lbl.Text = txt;
    end
end
function lockAxesToFixedRef(app)
    if isempty(app.I_fixed), return; end
    H = size(app.I_fixed,1); W = size(app.I_fixed,2);
    ax = [app.FixedAxes, app.MovingAxes, app.OverlayAxes];
    for a = ax
        if isempty(a) || ~isvalid(a), continue; end
        axis(a,'image');
        a.XLim = [0.5, W+0.5];
        a.YLim = [0.5, H+0.5];
        a.DataAspectRatio = [1 1 1];
    end
end
