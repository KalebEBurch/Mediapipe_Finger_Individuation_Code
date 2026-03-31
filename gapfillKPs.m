function [HAND_L_Corrected,HAND_R_Corrected,handDATA_corrected,BODY_Face_Corrected,BODY_L_Corrected,BODY_R_Corrected,bodyDATA_corrected,GAPS_HAND,GAPS_BODY] = gapfillKPs(myFolder,trialName)
% Use this function to gap fill keypoints from body and hand models.
%
% INPUTS
%    myFolder - path to the folder where pose data are stored. Class: char.
%    trialName - name given to your trial, which should be the start of the
%       filenames for your body and hand output .csv files. Class: char. 
%
% OUTPUTS
%    HAND_L_Corrected, HAND_R_Corrected - structures containing gap-filled
%       keypoints for the left and right hands
%    handDATA_corrected - matrix of all hand keypoints
%    BODY_L_Corrected, BODY_R_Corrected - structures containing gap-filled
%       left and right body keypoints
%    bodyDATA_corrected - matrix of all body keypoints
%    GAPS_HAND - structure with gap locations and gap sizes for the hands
%    GAPS_BODY - structure with gap locations and gap sizes for the body


%Motion filepaths (.csv files with marker paths)
bodyFilePath = [myFolder '\' trialName '_body_outputs.csv'];
handsFilePath = [myFolder '\' trialName '_hand_outputs.csv'];

%Extract kinematic data
[BODY,bodyDATA] = getBody(bodyFilePath);
[HAND_L,HAND_R,handDATA] = getHands(handsFilePath);

%Get count of hand keypoints for a single hand and both hands
nKP_hand = length(fieldnames(HAND_R));
nKP_hands = nKP_hand*2;

%Break down handDATA matrix into left and right hands
handDATA_L = handDATA(:,1:nKP_hand,:);
handDATA_R = handDATA(:,(nKP_hand+1):(2*nKP_hand),:);

%Get field names for hands and for body
fn_hand = fieldnames(HAND_L);
fn_bodyArm = fieldnames(BODY.L);
fn_bodyFace = fieldnames(BODY.Face);

%Get count of fieldnames (i.e. count of keypoints) for body model separate
%into side and face keypoints
nKP_bodySide = length(fieldnames(BODY.R));
nKP_bodyFace = length(fieldnames(BODY.Face));

%Get number of frames
nFrames = size(HAND_R.THUMB_1,1);

%Find missing keypoints
HAND_R_missing = findAllMissingKPs(HAND_R);
HAND_L_missing = findAllMissingKPs(HAND_L);
BODY_Face_missing = findAllMissingKPs(BODY.Face);
BODY_R_missing = findAllMissingKPs(BODY.R);
BODY_L_missing = findAllMissingKPs(BODY.L);

%Get time by loading video and extracting timestamps from all frames
%Load video
vidPath =  [myFolder '\' trialName '_tracked.avi'];
v = VideoReader(vidPath);
%Initialize to first frame
iFrame = 1;
%Initialize time array
tPose = NaN(nFrames,1);
%Iterate through each frame of video and store timestamps
while hasFrame(v)
    frame = readFrame(v);
    tPose(iFrame) = v.CurrentTime;
    iFrame = iFrame+1;
end

%Calculate frame rate for current video
fPose = 1/mean(diff(tPose));

%Define maximum acceptable frequency for gap filling. Assuming that body
%motions during all tasks are relatively slow at no more than 2Hz, we have
%set 2*2Hz = 4Hz as the maximum acceptable frequency for gap filling, so at
%30Hz we won't fill gaps larger than 30/4 = 7.5 frames (i.e. gaps of 7 
%frames or less will be filled).
fSlow = 4;
maxGapFillBody = fPose/fSlow;
maxGapFillHand = maxGapFillBody;

%If there are fewer than 2 frames, then skip gap filling.
if size(HAND_L.THUMB_1,1) < 2
    bodyDATA_corrected = bodyDATA;
    handDATA_corrected = handDATA;
    handDATA_L_corrected = handDATA_L;
    handDATA_R_corrected = handDATA_R;
    GAPS_HAND.L = [];
    GAPS_HAND.R = [];
    GAPS_BODY = [];
    HAND_L_Corrected = HAND_L;
    HAND_R_Corrected = HAND_R;
    BODY_Face_Corrected = BODY.Face;
    BODY_L_Corrected = BODY.L;
    BODY_R_Corrected = BODY.R;
else
    %Create matrices of all hand gaps and all body gaps
    is_error_both_hands = [HAND_L_missing HAND_R_missing];
    is_error_body = [BODY_Face_missing BODY_L_missing BODY_R_missing]; 
    %Identify edges of all gaps
    edges_of_KP_gaps = [zeros(1,nKP_hands); diff(is_error_both_hands)];

    %Find all frames where a marker goes missing (leading edge of the "is this
    %marker missing" signal)
    edges_of_missing_KPs_i = edges_of_KP_gaps(:,1);

    %Initalize corrected keypoints as duplicates of the original keypoints
    HAND_L_Corrected = HAND_L;
    HAND_R_Corrected = HAND_R;
    BODY_Face_Corrected = BODY.Face;
    BODY_L_Corrected = BODY.L;
    BODY_R_Corrected = BODY.R;
    
    %Fill left and right hand gaps
    [handDATA_L_corrected,GAPS_HAND_L] = fillKPs(handDATA_L,HAND_L_missing,maxGapFillHand);
    [handDATA_R_corrected,GAPS_HAND_R] = fillKPs(handDATA_R,HAND_R_missing,maxGapFillHand);
    %Fill body gaps
    [bodyDATA_corrected,GAPS_BODY] = fillKPs(bodyDATA,is_error_body,maxGapFillBody);
    
    %Store hand gap data
    GAPS_HAND.L = GAPS_HAND_L;
    GAPS_HAND.R = GAPS_HAND_R;
    
    %Concatenate left and right hand data into a single matrix
    handDATA_corrected = cat(2,handDATA_L_corrected,handDATA_R_corrected);

end

%Put corrected coordinates into structure form
%HAND
for iHand = 1:nKP_hand
    HAND_L_Corrected.(fn_hand{iHand}) = squeeze(handDATA_L_corrected(:,iHand,:));
    HAND_R_Corrected.(fn_hand{iHand}) = squeeze(handDATA_R_corrected(:,iHand,:));
end
%BODY FACE
for iFace = 1:nKP_bodyFace
    BODY_Face_Corrected.(fn_bodyFace{iFace}) = squeeze(bodyDATA_corrected(:,iFace,:));
end
%BODY SIDE
for iArm = 1:nKP_bodySide
    BODY_L_Corrected.(fn_bodyArm{iArm}) = squeeze(bodyDATA_corrected(:,nKP_bodyFace+iArm,:));
    BODY_R_Corrected.(fn_bodyArm{iArm}) = squeeze(bodyDATA_corrected(:,nKP_bodyFace+iArm+nKP_bodySide,:));
end
  
%Store time
HAND_L_Corrected.Time = tPose;
HAND_R_Corrected.Time = tPose;
BODY_Face_Corrected.Time = tPose;
BODY_L_Corrected.Time = tPose;
BODY_R_Corrected.Time = tPose;

%% HELPER FUNCTIONS

function [DATA_corrected,GAPS] = fillKPs(DATA,is_error,maxGapFill)

    %Get count of keypoints and count of frames
    nKP = size(DATA,2);
    nFrames = size(DATA,1);
    
    %Compute edges of keypoint gaps
    edges_of_KP_gaps = [zeros(1,nKP); diff(is_error)];
    
    %Find all frames where a marker goes missing (leading edge of the "is this
    %marker missing" signal)
    edges_of_missing_KPs_i = edges_of_KP_gaps(:,1);
    
    %Initialize corrected data as a duplicate of the original data
    DATA_corrected = DATA;
    
    %Define count of frames to use on left and right side for spline fit
    nSpline = 10;
    
    %Get just the x data
    xDATA = DATA(:,:,1);
    %Identify missing data at frames where x is zero
    missingData = (xDATA==0);
    %Get subscripts to missing data
    XYZ_missingData = repmat(missingData,1,1,3);
    %Set missing data to NaN
    DATA_corrected(XYZ_missingData) = NaN;
    %Identify all keypoints that have errors
    is_errant_KP = max(is_error);
    idx_is_errant_KP = find(is_errant_KP);
    %Get count of errant keypoints
    nErrantKP = length(idx_is_errant_KP);
        
    %Get edges of missing keypoints
    edges_of_missing_KPs_i = edges_of_KP_gaps(:,1);
    
    %Identify the beginning and end of each gap
    idx_leadEdges = find(edges_of_missing_KPs_i==1);
    idx_trailEdges = find(edges_of_missing_KPs_i==-1);

    %Set minimum threshold of 50% frames visible
    visThreshold = 0.5;
    
    % If idx_trailEdges empty, don't fill gaps and return empty data for 
    % all fields of the GAP structure
    if isempty(idx_leadEdges) | isempty(idx_trailEdges)
        GAPS.GapSizes = [];
        GAPS.GapStarts = [];
    else
        % If the first trail edge is before the first lead edge (i.e. the 
        % first  gap starts on the first frame), remove the first trail 
        % edge (don't fill the initial gap)
        if idx_trailEdges(1) < idx_leadEdges(1)
            idx_trailEdges = idx_trailEdges(2:end);
        end
          
        %If idx_trailEdges is now empty, don't fill gaps and return empty
        %data for all fields of the GAP structure
        if isempty(idx_trailEdges)
            GAPS.GapSizes = [];
            GAPS.GapStarts = [];
        else
    
            % If the last lead edge is after the last trail edge (i.e. the 
            % last gap ends on the last frame), remove the last lead edge 
            % (don't fill the last gap)
            if idx_leadEdges(end) > idx_trailEdges(end)
                idx_leadEdges = idx_leadEdges(1:end-1);
            end
        
            %Compute gap sizes
            gap_sizes = idx_trailEdges - idx_leadEdges;
            
            %Iterate through each KP that has gaps and fill the gaps
            for i = 1:nErrantKP
                %Get current KP
                iKP = idx_is_errant_KP(i);
            
                %Check is the keypoint visible more than 50% of the time
                nMissing_iKP = sum(DATA(:,iKP,1)==0);
                isVisible = (nMissing_iKP/nFrames) < visThreshold;
            
                %Only fill gaps if the keypoint meets visibility criterion 
                if isVisible
                
                    %Get count of missing frames
                    nMiss_i = length(idx_trailEdges);
             
                    for jMiss = 1:nMiss_i
                
                        %Get current gap size
                        gapSize = gap_sizes(jMiss);
            
                        %Get current leading and trailing edge
                        idx_leadEdge_j = idx_leadEdges(jMiss);
                        idx2_trailEdge_j = find(idx_trailEdges>idx_leadEdge_j,1);
                        idx_trailEdge_j = idx_trailEdges(idx2_trailEdge_j);
                        %Get current gap frame range 
                        range_j = idx_leadEdge_j:(idx_trailEdge_j-1);
            
                        %Define frames on either side of gap to use for
                        %gap-filling
                        fitPTs_start = max([1 idx_leadEdge_j-nSpline]);
                        fitPTs_end = min([nFrames idx_trailEdge_j+nSpline-1]);
                        fitPts = [fitPTs_start:(idx_leadEdge_j-1) idx_trailEdge_j:fitPTs_end];
                
                        %Get KP coordinates to be used for gap filling
                        KP_j_x = DATA_corrected(fitPts,iKP,1);
                        KP_j_y = DATA_corrected(fitPts,iKP,2);
                        KP_j_z = DATA_corrected(fitPts,iKP,3);
                
                        %Only fill gaps if they are less than or equal to
                        %the max gap fill size
                        if gapSize>=maxGapFill
                            gap_fill_x = NaN(size(range_j));
                            gap_fill_y = gap_fill_x;
                            gap_fill_z = gap_fill_x;
                        else
                            %Use Modified Akima Interpolation to estimate
                            %coordinates over gap
                            gap_fill_x = makima(fitPts,KP_j_x,range_j);
                            gap_fill_y = makima(fitPts,KP_j_y,range_j);
                            gap_fill_z = makima(fitPts,KP_j_z,range_j);
                        end
                
                        %Fill gap with interpolated data
                        DATA_corrected(range_j,iKP,1) = gap_fill_x;
                        DATA_corrected(range_j,iKP,2) = gap_fill_y;
                        DATA_corrected(range_j,iKP,3) = gap_fill_z;
                
                    end
                
                end
            
            end
            
            %Store gap sizes and gap starts
            GAPS.GapSizes = gap_sizes;
            GAPS.GapStarts = idx_leadEdges;
        
            %If no gaps were found, return GAPS as empty 
            if ~exist("GAPS",'var')
                GAPS.GapSizes = [];
                GAPS.GapStarts = [];
            end

        end    
    
    end

end


function all_missing_KPs = findAllMissingKPs(structKP)
    %This function uses the findMissingKPs function to find all keypoint
    %gaps in a structure

    %Find outliers for all keypoints
    all_missing_KPs_struct = structfun(@findMissingKPs,structKP,'UniformOutput',false);
    %Identify all frames in which at least one frame has an outlier/error 
    all_missing_KPs = convertStruct2Matrix(all_missing_KPs_struct);
end


function ERROR_ARRAY_mat = convertStruct2Matrix(ERROR_ARRAY)
    %This function is used to convert a structure to a matrix.

    %convert structure to cell
    ERROR_ARRAY_cell = struct2cell(ERROR_ARRAY);
    %convert cell to matrix
    ERROR_ARRAY_mat  = [ERROR_ARRAY_cell{:}];
end


function missingKPs = findMissingKPs(coordinates)
    %This function is used to identify missing KPs in a matrix

    %Look at just the x dimension
    Px = coordinates(:,1);
    %Find missing keypoints by identifying frames where x is zero
    missingKPs = (Px == 0);
end


function [BODY,DATAr] = getBody(bodyFilePath)
%This function is used to get body keypoint coordinates from a video

    %Extract kinematic data
    [DATA,headers] = getPoseData(bodyFilePath);
    
    %Get headers
    headersSide = {'SHOULDER','ELBOW','WRIST','PINKY','INDEX','HIP'};
    headersSideShort = {'SH','ELB','WRIST','PINKY','INDEX','HIP'};
    nHeadersSide = length(headersSideShort);
    headersL = cellfun(@(x) strcat('LEFT_',x),headersSide,'UniformOutput',false);
    headersR = cellfun(@(x) strcat('RIGHT_',x),headersSide,'UniformOutput',false);
    
    %Get keypoint count
    nKPs = length(headersL)*2 + 1;
    %Get count of spatial dimensions
    nDimension = sum(startsWith(headers,'NOSE'));
    
    %Reshape hand data matrix to 3 dims (nFrames x nKPs_bothHands x nDimensions)
    DATAr = NaN([size(DATA,1) nKPs nDimension]);
    
    %Store nose position
    BODY.Face.Nose = DATA(:,1:nDimension);
    DATAr(:,1,:) = DATA(:,1:nDimension);
    
    %Store all other positions
    for i=1:nHeadersSide
        %Get header to current keypoint for left and right side
        header_iL = startsWith(headers,headersL{i});
        header_iR = startsWith(headers,headersR{i});

        %Store current keypoint for both right and left side in BODY struct
        BODY.L.(headersSideShort{i}) = DATA(:,header_iL);
        BODY.R.(headersSideShort{i}) = DATA(:,header_iR);
    
        %Store current keypoint positions in DATAr for both left and right
        DATAr(:,i+1,:) = DATA(:,header_iL);
        DATAr(:,i+nHeadersSide+1,:) = DATA(:,header_iR);
    end
end


function [L_Hand,R_Hand,handDATAr] = getHands(handsFilePath)
%This function is used to get hand keypoint coordinates from a video

    %Get hand keypoints from csv file
    [handDATA,headers] = getPoseData(handsFilePath);
    
    %Get count of spatial dimensions
    nDim = sum(startsWith(headers,'L_THUMB_1'));
    
    %Get count of kehypoints
    nTotalKP = length(headers)/nDim;
    %Divide by 2 to get count of keypoints per hand
    nHandKP = nTotalKP/2;

    %Reshape hand data matrix to 3 dims (nFrames x nKPs_bothHands x nDimensions)
    handDATAr = NaN([size(handDATA,1) nTotalKP nDim]);
    
    %Iterate through each keypoint and store position data
    for i = 1:nHandKP
        %Identify first column of data for current keypoint
        idx_KP_i = nDim*i;
        %Identify all columns for current keypoint for both left and right
        %hand
        range_KP_iL = idx_KP_i-nDim+1:idx_KP_i;
        range_KP_iR = range_KP_iL + nHandKP*nDim;
    
        %Get current keypoint name
        KP_i = headers{idx_KP_i}(3:end-2);
    
        %Store current keypoint data in L_Hand and R_Hand structures
        L_Hand.(KP_i) = handDATA(:,range_KP_iL);
        R_Hand.(KP_i) = handDATA(:,range_KP_iR);
    
        %Store current keypoint data in handDATAr
        handDATAr(:,i,:) = handDATA(:,range_KP_iL);
        handDATAr(:,i+nHandKP,:) = handDATA(:,range_KP_iR);
    end
end


function [DATA,headers] = getPoseData(filepath)
%This function accepts a path to a csv file with pose data and exports a
%matrix with X and Y data as well as headers

    %Assumed frame width and frame height of video data
    w = 1;
    h = 1;
    
    %Import csv data
    A = importdata(filepath);
    DATA = A.data;
    headers = A.colheaders;
    
    %Correct position and orientation of origin by subtracting y-values from
    %frame height
    DATA(:,1:3:end) = w*DATA(:,1:3:end);
    DATA(:,2:3:end) = h-h*DATA(:,2:3:end);
    DATA(:,3:3:end) = w*DATA(:,3:3:end);
end


end