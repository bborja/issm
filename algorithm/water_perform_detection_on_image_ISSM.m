function detector = water_perform_detection_on_image_ISSM(detector, im, pUnknownFactor, varargin)
    detector_state = detector.detector_state;        

    [detector_state, sel_xy, objs, maskedSea] = detect_edge_of_sea_simplified_ISSM(detector_state, im, pUnknownFactor, varargin{:});
    
    % format output
    detector.detector_state = detector_state ;
    detector.objs = objs ;
    detector.sel_xy = sel_xy ;
    detector.maskedSea = maskedSea ;
    detector.is_initialized=true;

end


