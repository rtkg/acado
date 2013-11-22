%Acado for Matlab helper. Internal use only.
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%    Developed within the Optimization in Engineering Center (OPTEC) under
%    supervision of Moritz Diehl. All rights reserved.
%
%    ACADO Toolkit is free software; you can redistribute it and/or
%    modify it under the terms of the GNU Lesser General Public
%    License as published by the Free Software Foundation; either
%    version 3 of the License, or (at your option) any later version.
%
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%    Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ACADO Toolkit; if not, write to the Free Software
%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
%    Author: David Ariens
%    Date: 2010
% 
classdef AcadoMatlab < handle
    properties (SetAccess='protected')
        %setVariables
        t = {};     % time (can only be one element)
        x = {};     % diff states
        u = {};     % controls
        p = {};     % parameters
        w = {};     % disturbances
        z = {};     % alg states
        ints = {};  % intermediate states
        in = {};    % mexInput
    
        %instructions
        instructionList = {};
    
        problemname = '';
        fileMEX = '';

    end
    
    methods
        function obj = AcadoMatlab(varargin)

        end
        
        % Add instruction (differentialequation, ocp,... to list)
        function addInstruction(obj, set)
            obj.instructionList{length(obj.instructionList)+1} = set;
        end
        
        % Add time
        function addT(obj, set)
            if (~isempty(obj.t))
                error('You can only add TIME once.');
            end
            
            obj.t{1} = set;
        end
        
        % Add differential state
        function addX(obj, set)
            
            for i=1:length(obj.x)
                if (strcmp(obj.x{i}.name, set.name))
                   error('The differential state you are trying to add already exists.'); 
                end
            end
            
            obj.x{length(obj.x)+1} = set;
        end
        
        % Add algebraic state
        function addZ(obj, set)
            
            for i=1:length(obj.z)
                if (strcmp(obj.z{i}.name, set.name))
                   error('The algebraic state you are trying to add already exists.'); 
                end
            end
            
            obj.z{length(obj.z)+1} = set;
        end
        
        % Add control
        function addU(obj, set)
            
            for i=1:length(obj.u)
                if (strcmp(obj.u{i}.name, set.name))
                   error('The control you are trying to add already exists.'); 
                end
            end
            
            obj.u{length(obj.u)+1} = set;
        end
        
        % Add parameter
        function addP(obj, set)
            
            for i=1:length(obj.p)
                if (strcmp(obj.p{i}.name, set.name))
                   error('The parameter you are trying to add already exists.'); 
                end
            end
            
            obj.p{length(obj.p)+1} = set;
        end
        
        % Add disturbance
        function addW(obj, set)
            
            for i=1:length(obj.w)
                if (strcmp(obj.w{i}.name, set.name))
                   error('The disturbance you are trying to add already exists.'); 
                end
            end
            
            obj.w{length(obj.w)+1} = set;
        end
        
        % Add mex input
        function addIn(obj, set)
            
            for i=1:length(obj.in)
                if (strcmp(obj.in{i}.name, set.name))
                   error('The mex input you are trying to add already exists.'); 
                end
            end
            
            obj.in{length(obj.in)+1} = set;
        end
        
        % Add intermediate state
        function addIntS(obj, set)
            
            for i=1:length(obj.ints)
                if (strcmp(obj.ints{i}.name, set.name))
                   error('The intermediate state you are trying to add already exists.'); 
                end
            end
            
            obj.ints{length(obj.ints)+1} = set;
        end
        
        generateCPP(obj);
        getCPPheader(obj);
        getCPPbody(obj);
        getCPPfooter(obj);
        getCPPlefthandout(obj, nameB, name, out);
        
    end
    
end