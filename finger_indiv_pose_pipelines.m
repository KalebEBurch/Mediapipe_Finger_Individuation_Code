function RESULTS = finger_indiv_pose_pipelines(HAND)
% This function applies pipelines for finger individuation analysis on
% keypoint data from Mediapipe pose estimation applied to a digital video. 
% The function accepts a structure for hand keypoint coordinates created 
% by the correct_keypoints script. It outputs a stucture with finger 
% individuation results for each finger.
%
% INPUTS
% - HAND: a structure with subfields corresponding to 3D keypoint 
% coordinates stored as doubles of size (frame count) x 3. Additionally,
% one subfield contains a time series vector for the corresponding video.
%
% OUTPUTS
% - RESULTS: a structure which contains subfields corresponding to the
% individuation index (II), components of individuation index
% (COMPONENTS), indices to finger tap events (IDX_TAP), and the time series
% vector (TIME). The II, COMPONENTS, and IDX_TAP, fields are all structures
% with subfields containing doubles corresponding to results for each 
% finger (index, middle, ring, and pinky).

%Get count of frames
nFrames_i = length(HAND.THUMB_1);

%If there are fewer than 3 frames of data, set Results to NaN
if nFrames_i < 3
    RESULTS = NaN;
else
    
    %Get time series vector
    tPose = HAND.Time;

    %Compute average frame rate/sample frequency (usually near 30Hz)
    global fPose
    fPose = mean(1./diff(tPose));

    %Set lowpass filter cutoff frequency to 5 Hz
    cf = 5;

    %Set number of frames for median filter
    nMed = 11;
    %Apply median filter to thumb coordinates
    referenceTip = medfilt1(HAND.THUMB_4,nMed);

    %Apply lowpass Butterworth filter to thumbtip and fingertip coordinates
    Reference_F = myFilter(referenceTip,cf,4);
    HAND_F = structfun(@(x) myFilter(x,5,2),HAND,'UniformOutput',false);
    IndexTip_F = HAND_F.INDEX_4;         
    MiddleTip_F = HAND_F.MIDDLE_4; 
    RingTip_F = HAND_F.RING_4; 
    PinkyTip_F = HAND_F.PINKY_4;

    %Calculate relative position of fingertips to thumbtip
    indexRel = IndexTip_F - Reference_F;
    middleRel = MiddleTip_F - Reference_F;
    ringRel = RingTip_F - Reference_F;
    pinkyRel = PinkyTip_F - Reference_F;

    %Calculate norm of fingertip-to-thumbtip vectors
    indexTipNorm = vecnorm(indexRel,2,2);
    middleTipNorm = vecnorm(middleRel,2,2);
    ringTipNorm = vecnorm(ringRel,2,2);
    pinkyTipNorm = vecnorm(pinkyRel,2,2);

    %Zero and normalize each signal using max and min. Only consider
    %signals after the first 25% of the trial and before the last 25% of
    %the trial in order to avoid extreme values that may occur due to hand
    %movement at the start and end of the trial.

    %First calculate number of frames that corresponds to 25% of data
    nClip = round(nFrames_i/4);
    %Define set of frames that correspond with middle 50% of data
    frames_norm = nClip+1:nFrames_i-nClip;
    %Zero and normalize each signal
    indexTipFinal = normAndZero(indexTipNorm,frames_norm);
    middleTipFinal = normAndZero(middleTipNorm,frames_norm);
    ringTipFinal = normAndZero(ringTipNorm,frames_norm);
    pinkyTipFinal = normAndZero(pinkyTipNorm,frames_norm);

    %Put all signals into a single matrix
    allDisplacements = [indexTipFinal middleTipFinal ringTipFinal pinkyTipFinal];

    %Set minimum peak width
    t_minPW = 0.125;
    minPW = round(fPose*t_minPW);

    %Identify taps
    TAPS = IDtaps(allDisplacements,minPW);
    %Calculate individuation index and components for each repetition
    [II,COMPONENTS] = calcII(allDisplacements,TAPS);

    %Store II results by finger
    II_index = II(:,1);
    II_middle = II(:,2);
    II_ring = II(:,3);
    II_pinky = II(:,4);

    %Store components by finger
    COMPONENTS_index = squeeze(COMPONENTS(:,1,:));
    COMPONENTS_middle = squeeze(COMPONENTS(:,2,:));
    COMPONENTS_ring = squeeze(COMPONENTS(:,3,:));
    COMPONENTS_pinky = squeeze(COMPONENTS(:,4,:));

    %%%%%%%%%%%%%%%%%%%%%% CLEAN DATA BY REMOVING INF %%%%%%%%%%%%%%%%%%%%%

    II_index_clean = II_index(~isinf(II_index));
    II_middle_clean = II_middle(~isinf(II_middle));
    II_ring_clean = II_ring(~isinf(II_ring));
    II_pinky_clean = II_pinky(~isinf(II_pinky));
    
    COMPONENTS_index_clean = COMPONENTS_index(~isinf(COMPONENTS_index(:,1)),:);
    COMPONENTS_middle_clean = COMPONENTS_middle(~isinf(COMPONENTS_middle(:,1)),:);
    COMPONENTS_ring_clean = COMPONENTS_ring(~isinf(COMPONENTS_ring(:,1)),:);
    COMPONENTS_pinky_clean = COMPONENTS_pinky(~isinf(COMPONENTS_pinky(:,1)),:);

    %%%%%%%%%%%%%%% STORE ALL RESULTS IN A STRUCTURE %%%%%%%%%%%%%%%%%%%%%%

    RESULTS.DISPLACEMENTS.Index = allDisplacements(:,1);
    RESULTS.DISPLACEMENTS.Middle = allDisplacements(:,2);
    RESULTS.DISPLACEMENTS.Ring = allDisplacements(:,3);
    RESULTS.DISPLACEMENTS.Pinky = allDisplacements(:,4);

    RESULTS.II.Index = II_index_clean;
    RESULTS.II.Middle = II_middle_clean;
    RESULTS.II.Ring = II_ring_clean;
    RESULTS.II.Pinky = II_pinky_clean;

    RESULTS.COMPONENTS.Index = COMPONENTS_index_clean;
    RESULTS.COMPONENTS.Middle = COMPONENTS_middle_clean;
    RESULTS.COMPONENTS.Ring = COMPONENTS_ring_clean;
    RESULTS.COMPONENTS.Pinky = COMPONENTS_pinky_clean;

    RESULTS.IDX_TAPS.Index = find(TAPS==1);
    RESULTS.IDX_TAPS.Middle = find(TAPS==2);
    RESULTS.IDX_TAPS.Ring = find(TAPS==3);
    RESULTS.IDX_TAPS.Pinky = find(TAPS==4);

    RESULTS.TIME = tPose;

end


%% HELPER FUNCTION

function xF = myFilter(x,cf,nOrder)
% INPUTS
% x:  signal
% cf: cut-off frequency
% nOrder: filter order

    xNonNaNs = ~isnan(x(:,1));
    idxNonNaNs = find(xNonNaNs);

    x2 = x(idxNonNaNs,:);
    
    %Create 2nd order lowpass butterworth filter
    Wn = cf/fPose;
    [B,A] = butter(nOrder,Wn,'low');
    
    x2F = filtfilt(B,A,x2);

    xF = x;
    xF(idxNonNaNs,:) = x2F;

end


function xNZ = normAndZero(x,frames)
%Use this function to normalize and zero a signal within a designated
%frame range
%INPUTS
% x - input signal
% frames - set frame range to apply zeroing and normalization
%OUTPUT
% xNZ - the normalized and zeroed signal

    %Identify minimum value of frames in designated frame range
    x_floor = min(x(frames,:));
    %Zero the entire signal to this value
    xZ = x - x_floor;
    %Identify ceiling of zeroed signal in designated frame range
    x_ceil = max(xZ(frames,:));
    %Normalize the entire signal to this value
    xNZ = xZ./x_ceil;
end


function TAPS = IDtaps(D,minPW)
% Use this function to identify taps from the finger individuation task.
% Effectively, this function finds relevant minima of overlapping cyclical
% signals, all of which have magnitudes of roughly 0 to 1.
%
% INPUTS
% D - matrix of time series data for all signals.
% minPW - minimum peak width used in the peak detection step
%
% OUTPUTS
% TAPS: a vector indicating which taps (specified as integers 1, 2, ..., n)
% occur at the frames where they occur.
%    Example: If TAPS = [0 0 1 0 0 2 0 0 0 3], then finger 1 taps the thumb 
%    at frame 3 (i.e. signal 1 reaches a relevant local minima at frame 3), 
%    finger 2 taps the thumb at frame 6, and finger 3 taps the thumb at 
%    frame 10.
    
    %Find minimum value and corresponding finger at each frame
    [Dmin,~] = min(D,[],2);
    %Detrend the Dmin signal
    DminDetrend = detrend(Dmin,'omitnan');
    %Find local minima by inverting detrended Dmin signal. Minima are peaks in
    %the inverted signal. Only find peaks above zero (the detrended signal has
    %zero mean, so 0 corresponds with 50% scale). Only find peaks with half
    %prominence width greater than the specified value.
    [~,idxSumMins] = findpeaks(-DminDetrend,'MinPeakHeight',0,...
        'MinPeakWidth',minPW,'WidthReference','halfprom');
    %Identify which finger being tapped by identifying which is at the lowest
    %value at each minima
    [~,idxTap_test] = min(D(idxSumMins,:),[],2);
    %Separate out each tap by finger
    idxTaps_c1min = idxSumMins(idxTap_test==1);
    idxTaps_c2min = idxSumMins(idxTap_test==2);
    idxTaps_c3min = idxSumMins(idxTap_test==3);
    idxTaps_c4min = idxSumMins(idxTap_test==4);
    %Store finger tap classifications in TAPS array
    TAPS(idxTaps_c1min) = 1;
    TAPS(idxTaps_c2min) = 2;
    TAPS(idxTaps_c3min) = 3;
    TAPS(idxTaps_c4min) = 4;

end


function [II,COMPONENTS,TAPS2] = calcII(D,TAPS)
% This function identifies each time each a finger taps the thumb.
%
% INPUTS
% D - matrix of time series data for all signals.
%
% OUTPUTS
% II - individuation index for each finger tap repetition
% COMPONENTS - displacement of non-active fingers scaled by active finger
% for each repetition
% TAPS2 - updated vector of tap indices with false taps removed

    %Identify taps for finger of interest and count them
    idx_taps = find(TAPS);

    %Get total finger tap count by identifying nonzero elements of idx_taps
    nTapsTot = nnz(idx_taps);
    %Get count of taps for each individual finger
    nTaps1 = nnz(TAPS==1);
    nTaps2 = nnz(TAPS==2);
    nTaps3 = nnz(TAPS==3);
    nTaps4 = nnz(TAPS==4);
    %Get the largest finger tap count
    nTaps_Max = max([nTaps1 nTaps2 nTaps3 nTaps4]);

    nF = 4; %number of fingers 

    %Initialize array of individuation indices
    II = NaN(nTaps_Max,nF);
    COMPONENTS = NaN(nTaps_Max,nF,nF-1);

    %Duplicate TAPS to update when taps are false.
    TAPS2 = TAPS;

    %Initialize variable prev_tap_i and set to frame 1.
    idx_prev_tap_i = 1;
    for i = 1:nTapsTot
        %Identify index to last time a finger was tapped before this
        %finger was tapped
        idx_tap_i = idx_taps(i);

        %Set frame range from previous finger tap to current finger tap
        idxFrames_i = idx_prev_tap_i+1:idx_tap_i;

        %Get index to the active finger
        idxFinger = TAPS(idx_tap_i);
        %Get set of other fingers by removing active finger from full set
        other_fingers = setxor(idxFinger,1:nF);

        %Get count of taps for current finger not including current tap
        nII_finger_current = sum(~isnan(II(:,idxFinger)));
        %Add 1 to get proper index for current finger tap
        idxII = nII_finger_current + 1;
        
        %As long as a tap is not repeated, proceed to calculate II
        if idx_tap_i~=idx_prev_tap_i

            %Get position data for current finger over current interval
            pos_i = D(idxFrames_i,idxFinger);
            %Calculate max negative displacement over current interval
            dist_i = maxNegDisp(pos_i);

            %Get position data for other fingers
            pos_other_i = D(idxFrames_i,other_fingers);
            %Compute max negative displacements for other fingers
            dist_other1_i = maxNegDisp(pos_other_i(:,1));
            dist_other2_i = maxNegDisp(pos_other_i(:,2));
            dist_other3_i = maxNegDisp(pos_other_i(:,3));
            %Put other finger displacements into single matrix
            dist_other_i = [dist_other1_i dist_other2_i dist_other3_i];

            %Normalize displacement of other fingers to 
            norm_dist_other_i = dist_other_i/dist_i;
    
            %Calculate individuation index. Don't subtract 1 as is done in
            %other papers because this is meant to cancel out the active
            %finger's displacement, which we have already excluded
            II_i = 1 - ((sum(norm_dist_other_i))/(nF-1));
            
            %Store individuation index
            II(idxII,idxFinger) = II_i;
    
            %Store components
            COMPONENTS(idxII,idxFinger,:) = norm_dist_other_i;
    
            %Update TAPS2 if II_i is NaN
            if isnan(II_i)
                TAPS2(idx_tap_i) = 0;
            end
    
            %Set current tap to previous tap before next iteration
            idx_prev_tap_i = idx_tap_i;

        end
    end
end


function maxDist = maxNegDisp(d)
% Use this function to calculate the maximum negative displacement of a
% signal
%
% INPUTS
% d: a signal that varies over time
%
% OUTPUT
% maxDist: the maximum negative displacement of the signal

    %Calculate maximum negative displacement over entire interval
    [maxDist,idx_d_max] = calcMaxDisp(d);
    
    %If a maximum displacement of zero is returned, look for the maximum
    %negative displacement before and after the maximum value.
    if maxDist == 0
        %Calculate max negative displacement before and after
        maxDistBefore = calcMaxDisp(d(1:idx_d_max-1));
        maxDistAfter = calcMaxDisp(d(idx_d_max:end));
        %The desired max negative displacment is the greater of these two
        maxDist = max([maxDistBefore maxDistAfter]);
    end

end

function [maxDist,idx_d_max] = calcMaxDisp(d)
% This function identifies the absolute maximum and minimum of a signal and
% compare when they occur in the signal. If the minimum occurs after the
% maximum, then return the difference between the two, otherwise, return 0.

    %Find the absolute max and min on the interval
    [d_max,idx_d_max] = max(d);
    [d_min,idx_d_min] = min(d);
    
    %If absoulte minimum occurs after absoulte maximum
    %maxDist is the difference between the two; otherwise, it is zero.
    if idx_d_min > idx_d_max
        maxDist = d_max - d_min;
    else
        maxDist = 0;
    end
end


end