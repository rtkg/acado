%The class DifferentialEquation allows to setup and evaluate differential equations (ODEs) based on SymbolicExpressions.
%
%  Usage:
%    >> DifferentialEquation();
%    >> DifferentialEquation(tStart, tEnd);
%
%  Parameters:
%    tStart 	start of the time horizon of the diff eq     [NUMERIC / PARAMETER]
%    tEnd       end of the time horizon of the diff eq       [NUMERIC / PARAMETER]
%
%  Example:
%    >> f = acado.DifferentialEquation();
%    >> f = acado.DifferentialEquation(0.0, 10.0);
%
%  See also:
%    acado.DifferentialEquation.add             Adds a differential equation in symbolic syntax
%    acado.DifferentialEquation.linkMatlabODE   Links a matlab black box model
%    acado.DifferentialEquation.linkMatlabDAE
%    acado.DifferentialEquation.linkCFunction   Links a c function
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
%    Date: 2009-2010
% 
classdef DifferentialEquation < acado.Function    
    properties(SetAccess='private')
        tStart;
        tEnd;
        
        %MATLAB ODE/DAE/Jac
        matlabODE_fcnHandle = '';
        matlabDAE_fcnHandle = '';
        matlabJacobian_fcnHandle = '';
        matlablinkcount = 0;
        
        cfunction_file = '';
        cfunction_function = '';
        
        % Differential eq
        differentialList = {};
    end
    
    methods
        function obj = DifferentialEquation(varargin)
           checkActiveModel;
           
           if (nargin == 2)

                if (isa(varargin{1}, 'acado.Expression'))
                    obj.tStart = varargin{1};
                else
                    obj.tStart = acado.DoubleConstant(varargin{1});    
                end
                
                if (isa(varargin{2}, 'acado.Expression'))
                    obj.tEnd = varargin{2};
                else
                    obj.tEnd = acado.DoubleConstant(varargin{2});    
                end
                
            end
            
            global ACADO_;
            obj.matlablinkcount = ACADO_.count_function;
            
        end
        
        
        linkMatlabODE(obj, varargin)
        linkMatlabJacobian(obj, fcnHandle)
        linkCFunction(obj, fcnfile, fcnname)
        ODE(obj, varargin)
        add(obj, varargin)
        getInstructions(obj, cppobj, get)
        
        function r = getHeader(obj)
            
            if (~isempty(obj.tStart) && ~isempty(obj.tEnd))
                r = sprintf('DifferentialEquation %s(%s, %s)',obj.name, obj.tStart.name, obj.tEnd.name);

            else
                r = sprintf('DifferentialEquation %s',obj.name);
            end
            
        end
    end
    
end

