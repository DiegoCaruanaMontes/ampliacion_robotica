function y = RMSE(data)
n = data.i;
path = data.path;
true_path = data.true;

y = (true_path-path).^2;
y = sqrt(sum(y,2)/n);
end
