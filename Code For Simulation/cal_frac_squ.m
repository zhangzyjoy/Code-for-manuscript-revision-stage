function y = cal_frac_squ(x, a, b)
% 计算 sign(x^a) * |x^a|^(1/b)
% 使用对数变换避免大数溢出

if a < 0 && abs(norm(x, 2)) <= 1e-03
    y = zeros(size(x));
else
    % 对输入值取绝对值，避免负数的分数幂问题
    abs_x = abs(x);
    
    % 处理正数和负数的符号问题
    sign_x = sign(x);
    
    % 使用对数变换计算 |x|^a
    % 注意：对于 x=0 的情况需要特殊处理
    zero_mask = abs_x == 0;
    non_zero_mask = ~zero_mask;
    
    % 初始化结果矩阵
    y = zeros(size(x));
    
    % 对于非零元素，使用对数变换
    if any(non_zero_mask(:))
        log_abs_x = log(abs_x(non_zero_mask));
        
        % 计算 a * log(|x|) 然后取指数
        exponent_a = a * log_abs_x;
        
        % 检查是否可能溢出
        max_exp = log(realmax('double'));
        if any(exponent_a > max_exp)
            % 如果可能溢出，使用分段处理
            safe_mask = exponent_a <= max_exp;
            overflow_mask = exponent_a > max_exp;
            
            % 安全部分正常计算
            if any(safe_mask)
                temp_result = exp(exponent_a(safe_mask));
                % 计算 (1/b) 次幂
                y_temp = sign(x(non_zero_mask(safe_mask))) .^ a .* abs(temp_result) .^ (1/b);
                y(non_zero_mask(safe_mask)) = y_temp;
            end
            
            % 溢出部分使用对数变换计算最终结果
            if any(overflow_mask)
                % 直接计算 sign(x^a) * |x^a|^(1/b) 的对数形式
                % sign(x^a) = sign(x)^a
                % log(|x^a|^(1/b)) = (a/b) * log(|x|)
                log_result = (a/b) * log_abs_x(overflow_mask);
                
                % 恢复最终结果
                y_temp = sign(x(non_zero_mask(overflow_mask))) .^ a .* exp(log_result);
                y(non_zero_mask(overflow_mask)) = y_temp;
            end
        else
            % 没有溢出风险，正常计算
            temp_result = exp(exponent_a);
            y_temp = sign(x(non_zero_mask)) .^ a .* abs(temp_result) .^ (1/b);
            y(non_zero_mask) = y_temp;
        end
    end
    
    % 对于零元素，需要特殊处理
    if any(zero_mask(:))
        if a > 0
            y(zero_mask) = 0;  % 0^正数 = 0
        elseif a == 0
            y(zero_mask) = 1;  % 0^0 定义为 1（与MATLAB一致）
        else % a < 0
            % 当 a < 0 且 x = 0 时，结果为 Inf
            y(zero_mask) = Inf;
        end
    end
end

end
