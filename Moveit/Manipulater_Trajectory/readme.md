# rosrun 오류
```
[rosrun] couldn't find executable named {} below {path}
```

# 해결방법
## 모든 패키지 빌드할 수 있게 변경한 후
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
## *catkin_make* 실행
