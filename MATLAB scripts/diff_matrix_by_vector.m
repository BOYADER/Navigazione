function dR=diff_matrix_by_vector(R, x)
[row, col]=size(R);
n=length(x);
diff_R=[];
    for i=1:row
        test=[];

        for j=1:col
            for k=1:n
                a(k)=diff(R(i, j), x(k)); %derivata singolo elemento per x
            end
            test=[test, a];
        end
        diff_R=[diff_R; test];
    end
    dR=diff_R;
end