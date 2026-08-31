allData = cell(1, 10); % 데이터를 저장할 셀 배열
for i = 1:10
    fileName = sprintf('driving_hv%d_info_space', i);
    allData{i} = load(fileName); % 모든 파일 미리 로드
end