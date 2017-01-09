%CLASS_INTERFACE Example MATLAB class wrapper to an underlying C++ class
classdef PersonList_interface < handle
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = PersonList_interface(varargin)
            this.objectHandle = PersonList_interface_mex('new', varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            PersonList_interface_mex('delete', this.objectHandle);
        end

        %% UpdateDeltaT - updates the delta T
        function varargout = updateDeltaT(this, varargin)
            [varargout{1:nargout}] = PersonList_interface_mex('updateDeltaT', this.objectHandle, varargin{:});
        end
        
        function varargout = associateData(this, varargin)
            [varargout{1:nargout}] = PersonList_interface_mex('associateData', this.objectHandle, varargin{:});
        end
        function varargout = deleteMarkedTracklets(this, varargin)
            [varargout{1:nargout}] = PersonList_interface_mex('deleteMarkedTracklets', this.objectHandle, varargin{:});
        end
    end
end
