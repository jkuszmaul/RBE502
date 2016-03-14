classdef Kin_Model
    %Kin_Model: Kinematic Model of a serial robot manipulator
    
    properties
    
    end
    
    properties(SetAccess = private)
        q
        n_frames
        n_joints
        J
        inv_J
        inv_J0
    end
    
    properties(Access = private)
        T_matrices
        lambda_val = 0.001  %small positive real value used to remove singularities in inverse Jacobian
        lambda_inv_J    %inverse Jacobian with symbolic lambda to quickly recalculate when lambda changes
        sym_lambda = sym('lambda');
    end
    
    properties (Dependent)
        lambda
    end
    
    methods

        function value = T(obj,from,to)
            value = obj.T_matrices{from+1,to+1};
        end 
        
        function value = forward_kin(obj,q)
            value = subs(obj.T(0,obj.n_frames),obj.q,q);
        end
        
        function value = inv_kin(obj,H)
            error('function not implemented')
        end
        
        function obj = set.lambda(obj,value)
            obj.lambda_val = value;
            if ~isempty(obj.lambda_inv_J)
                obj.inv_J = simplify(vpa(subs(obj.lambda_inv_J,obj.sym_lambda,obj.lambda)));
            end
        end
        function value = get.lambda(obj)
            value = obj.lambda_val;
        end
    end
    
    methods(Static)
        function obj = fromH_TransChain(varargin)
            obj = Kin_Model;
            obj = obj.init(size(varargin{1},1));
            for i = 1:obj.n_Frames
                obj.T_matrices{i,i+1}=vpa(varargin{i});
            end
            obj=obj.calculateFromChain();
            obj=obj.calculateJacobian();
            obj=obj.calculatePesudoInvJacobian();
        end
        
        function obj=fromDH(DH_Matrix,jointVars)
            obj = Kin_Model;
            obj = obj.init(size(DH_Matrix,1));
            if nargin<2
                obj.q = sym('q',[obj.n_frames,1]);
            else
                obj.q = jointVars;
            end
            for i = 1:obj.n_frames
                obj.T_matrices{i,i+1}=simplify(vpa(H_Trans.fromDH(DH_Matrix(i,:)).H));
            end
            obj=obj.calculateFromChain();
            obj=obj.calculateJacobian();
            obj=obj.calculatePesudoInvJacobian();
        end
        
    end
    
    methods(Access = private)
        function obj = init(obj,frames)
            obj.n_frames=frames;
            obj.T_matrices=cell(frames+1,frames+1);
            for i = 1:frames+1
                obj.T_matrices{i,i} = eye(4);
            end
        end
        
        function obj = calculateFromChain(obj)
            for r = 1:size(obj.T_matrices,1)
               for c = 1:size(obj.T_matrices,2) 
                   if c>(r+1)
                       temp=eye(4);
                       for i=r:(c-1)
                           temp=temp*obj.T_matrices{i,i+1};
                       end
                       obj.T_matrices{r,c} = simplify(vpa(temp));
                   end
               end
            end
            
            for r = 1:size(obj.T_matrices,1)
               for c = 1:size(obj.T_matrices,2) 
                   if c<r
                       obj.T_matrices{r,c} = simplify(vpa(inv(obj.T_matrices{c,r})));
                   end
               end
            end
        end
        
        function obj = calculateJacobian(obj)
            obj.J=H_Trans(obj.T(0,size(obj.q))).getJacobian(obj.q);
        end
        
        function obj = calculatePesudoInvJacobian(obj,lambda)
            if nargin<2
                lambda = obj.lambda;
            end
            
            obj.lambda_inv_J = simplify(vpa(obj.J.'/(obj.J*obj.J.' + (obj.sym_lambda^2).*eye(max(size(obj.J))))));
            obj.inv_J0 = simplify(vpa(subs(obj.lambda_inv_J,obj.sym_lambda,0)));
            obj.lambda = lambda;
        end
    end
    
end

