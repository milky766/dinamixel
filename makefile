##################################################
# PROJECT: DXL Protocol 2.0 Current Control Example Makefile
##################################################

# ターゲット名を指定
TARGETS = current_control current_control2 error current

# SDKのディレクトリとオブジェクトファイルを保存するディレクトリを指定
DIR_DXL    = ../../..
DIR_OBJS   = .objects

# コンパイラとフラグの設定
CX          = g++
CXFLAGS     = -O2 -O3 -std=c++17 -DLINUX -D_GNU_SOURCE -Wall -I$(DIR_DXL)/include/dynamixel_sdk -m64 -g
LNKFLAGS    = -O2 -O3 -std=c++17 -DLINUX -D_GNU_SOURCE -Wall -I$(DIR_DXL)/include/dynamixel_sdk -m64 -g
LIBRARIES   = -ldxl_x64_cpp -lrt -lstdc++fs

# オブジェクトファイル用のディレクトリを作成
$(DIR_OBJS):
	mkdir -p $(DIR_OBJS)

# ターゲットファイルの作成
all: $(DIR_OBJS) $(TARGETS)

current_control: $(DIR_OBJS)/current_control.o
	$(CX) $(LNKFLAGS) $(DIR_OBJS)/current_control.o -o current_control $(LIBRARIES)

current_control2: $(DIR_OBJS)/current_control2.o
	$(CX) $(LNKFLAGS) $(DIR_OBJS)/current_control2.o -o current_control2 $(LIBRARIES)


error: $(DIR_OBJS)/error.o
	$(CX) $(LNKFLAGS) $(DIR_OBJS)/error.o -o error $(LIBRARIES)

current: $(DIR_OBJS)/current.o
	$(CX) $(LNKFLAGS) $(DIR_OBJS)/current.o -o current $(LIBRARIES)
	
# 個々のオブジェクトファイルの生成ルール
$(DIR_OBJS)/current_control.o: current_control.cpp
	$(CX) $(CXFLAGS) -c current_control.cpp -o $(DIR_OBJS)/current_control.o

$(DIR_OBJS)/current_control2.o: current_control2.cpp
	$(CX) $(CXFLAGS) -c current_control2.cpp -o $(DIR_OBJS)/current_control2.o


$(DIR_OBJS)/error.o: error.cpp
	$(CX) $(CXFLAGS) -c error.cpp -o $(DIR_OBJS)/error.o

$(DIR_OBJS)/current.o: current.cpp
	$(CX) $(CXFLAGS) -c current.cpp -o $(DIR_OBJS)/current.o

# 中間ファイルを削除するためのルール
clean:
	rm -rf $(TARGETS) $(DIR_OBJS) core *~ *.a *.so *.lo
