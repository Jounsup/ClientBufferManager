classdef manifest
    properties (SetAccess = public)
        FrameToWorldScale = 0;
        FrameToWorldRotation = 0;
        FrameToWorldTranslation = 0;
        startTime = 0;
        endTime = 0;
        duration = 100;
        startSegment = 0;
        segmentDuration = 1;
        degreesPerTileAt1m = 0.1;
        representationCount = 5;
        URLbase = '\\pyrrha';
        Rep = 0;
    end
    methods 
%         function A = manifest(FrameToWorldScale,FrameToWorldRotation,FrameToWorldTranslation,startTime,...
%                 endTime,duration,startSegment,segmentDuration,degreesPerTileAt1m,representationCount,URLbase,Rep)
%             A.FrameToWorldScale = FrameToWorldScale;
%             A.FrameToWorldRotation = FrameToWorldRotation;
%             A.FrameToWorldTranslation = FrameToWorldTranslation;
%             A.startTime = startTime;
%             A.endTime = endTime;
%             A.duration = duration;
%             A.startSegment = startSegment;
%             A.segmentDuration = segmentDuration;
%             A.degreesPerTileAt1m = degreesPerTileAt1m;
%             A.representationCount = representationCount;
%             A.URLbase = URLbase;
%             A.Rep = Rep;
%         end
    end
end