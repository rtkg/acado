%User-interface to formulate and solve optimal control problems and static NLPs. 
%
%  Usage:
%    >> OptimizationAlgorithm(ocp);
%
%  Parameters:
%    ocp 	    link to an OCP object                 [acado.OCP]
%
%  Example:
%    >> ocp = acado.OCP(0.0, 1.0, 20);
%    >> algo = acado.OptimizationAlgorithm(ocp);   
%
%  See also:
%    acado.OptimizationAlgorithm.initializeDifferentialStates
%    acado.OptimizationAlgorithm.initializeAlgebraicStates
%    acado.OptimizationAlgorithm.initializeControls
%    acado.OptimizationAlgorithm.initializeParameters
%    acado.OptimizationAlgorithm.initializeDisturbances
%    acado.OptimizationAlgorithm.set
%
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
%    Date: 2009
% 
classdef OptimizationAlgorithm < acado.OptimizationAlgorithmBase    
    properties(SetAccess='protected')

    end
    
    methods
        function obj = OptimizationAlgorithm(varargin)

            global ACADO_;
            ACADO_.count_optalgo = ACADO_.count_optalgo+1;
            obj.name = strcat(obj.name, num2str(ACADO_.count_optalgo));
            
            if (nargin == 1 && isa(varargin{1}, 'acado.OCP') )
                obj.ocp = varargin{1};   
            end

            ACADO_.helper.addInstruction(obj);
            
        end
        
        getInstructions(obj, cppobj, get)

        
    end
    
end

