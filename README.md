# Cimpact
a cpp program for finite element method of impact problem

# 更新日志

> 4月12日，16:24更新
* 1.最核心的问题，还是如何保证，nodeList中的节点与elementTable中的节点是指向相同地址的，也就是一样的

> 4月13号，7:25更新
*  1.通过声明class Node 已经能够解决一小部分error了，但最关键的施加边界条件的两个函数使用node时就会报错，还是找不到标示符

> 4月13号，15:25更新
*  1.添加了很多一维向量计算的子程序，如计算模，计算叉乘等;除此之外，使用了二维向量中LU分解来计算矩阵的逆，因此头文件jama_lu.h以后必须带上
* 2.核心计算不分，也就是位置更新，位移计算，应力应变结点力，以及约束外载的施加都写上去了，但还没有对每个子程序进行校核
* 3.不能整体校核，应该先对每个子过程进行试验，写出流程示意作为注释，再针对简单的算例进行试算

> 4月13号，23:34更新
* 1.解决了一个尿点bug，解析约束文件时，保存过约束name，再把它push到vector中，但无论如何都竟然不能保存name信息，百思不得其解，也是debug好几次才发现是name不匹配导致的。最后猛然想起BdConstraint写了复制构造函数，且漏掉了name的关键信息，赶紧加上，完美解决问题
	惊叹C++的严谨精细，看来不弄清楚语法底层细节，很难驾驭CPP
* 2.目前文件读取已经完全没问题，bug出在质量信息无法再node列中传递，但在element列表中的单元左右节点却有mass，问题是，计算mass赋值给的就是左右节点，没法给node列赋值，但左右节点并不与node列相关，解决方案应该是左右节点直接取自node列，而非在单元中重新定义
* 3.BCondition还是没解决
* 4.Ver_07.in是个检验程序非常好的算例，用这个泡泡

>4月14号，8:55更新
*  1.解决nodeList，nodeTable中的节点与elementTable，elementList的node不相关的根本，应该是搞清楚填充单元列时用的节点从哪儿来，是否关联
* 2.单元计算应力应变力，最终都落实到基层的左右节点上了。因此只要单元表中左右节点是有质量信息，就能计算，不需要单元上的节点与节点表一一对应
* 3.问题的根源是，一路下来又用List又用Map，导致传值混乱
* 4.写给转化函数，输入单元表，通过map的不重复单元性质，把单元表中的单元的左右节点给mapNode，再转给nodeList
* 5.重写一个给弹性材料用的杆单元
* 6.通过map的删除重复元素的功能，将传入的elementTable提炼到nodeTable1，再换位
* 7.最后存到nodeList中
* 8.错误	37	error C2678: 二进制“<”: 没有找到接受“const Node”类型的左操作数的运算符(或没有可接受的转换)	193	1

```cpp

	nodeList.clear();
	map<Node, int> temp_nodeTable1;
	for (auto element:elementTable)
	{
	temp_nodeTable1.insert({ *element.second.leftNode, element.first });
	temp_nodeTable1.insert({ *element.second.rightNode, element.first });
	}
	map<int, Node> temp_nodeTable2;
	for (auto element:temp_nodeTable1)
	{
	temp_nodeTable2.insert({ element.second, element.first });
	}
	for (int i = 0; i < temp_nodeTable2.size();i++)
	{
	nodeList.push_back(temp_nodeTable2[i]);
	}
	
```	
>4月14号，12：00更新
* 1.由于模式使用还不成熟，只能将杆单元中的材料全换成弹性的，进行V7试算;前一个文件还存在着问题
* 2.前期理解认为，数据和程序没能正确描述这个物理过程，但仔细分析程序发现，还是有很多玄机的：首先，apply加速度bc时，会把前期通过弹性变形计算的节点反力得到的加速度冲掉，这是在有了速度边界条件的情况下进行的；   也就意味着，没有速度加速度边界时，会通过弹性变形产生的加速度产地到另外的节点上，使之运动
* 3.这两个apply目的就是，在有速度加速度边界时，冲掉原来弹性反力产生的速度加速度，保持边界的约束，而没有这些边界时，就会通过反力来联动两个节点为一体

>4月15号，8:05更新
* 1.之前算法在解析控制信息时，都没有处理自动步长，也就是如果没有STEP信息该怎么办，今天解决了，能同时识别自动步长和固定步长
* 2.给每一个程序步写一个=======华丽丽分割线，太有用了，能很高效的定位到哪个程序块出问题，并能够高效排错
* 3.昨天好像忘记记了，调试时VS提供了实时变量检测的功能，并可以设置标签，直接显示在桌面上，debug效率很高
* 4.昨天写的  单元表节点迁移函数，无法适用到多单元中，今天更新；不再从单元列节点往节点列中迁移了，直接从节点表往节点列迁移，但保留原有迁移函数
* 5.新问题，质量分布出错，单元质量是对的，但分给节点时公共节点会累计，但存在nodeList的node并没有这种效果..
>4月15号，13:16更新
* 1.有时候问题可能出在理论计算上，比如今天计算Variable7的6号单元质量时，竟然把长度计算错误，原因是把坐标看错了，而程序计算完全正确，由于理论计算偏小，刚开始一个单元一个debug，没找出来，毕竟不愿意循环6次，之后干脆把程序计算的节点质量，单元质量全输出，一对比，就一下发现错误原因了。
* #由此看来，debug还是得与直接遍历输出结合起来,这不立马发现了新bug哈哈，享受bug
* 2.Variable的status方法出问题，没法提取出0时刻的加速度值，发现调试程序找一个很标准的示例很重要，V7找出来了迭代过程的bug，避开复杂的时间谱，Va找出了时间谱函数的错误，前提是基本迭代没问题
```cpp
	string filePath1 = "E:\\1dpl_node1.txt";
	string filePath2 = "E:\\2stress_element.txt";
	ofstream cout_dpl, cout_stress;
	cout_dpl.open(filePath1);
	cout_stress.open(filePath2);
	if (!cout_dpl.is_open() || !cout_stress.is_open())
	{
		cout << "file to open file1\n";
	}
	timestep = controlset.getTimeStep(0);
	int stepNum = 0;
//获取时间步出错
	if (controlset.autoStep)
	{
		autostep = true;
	} else timestep = controlset.getTimeStep(0);
```
>
* 4.controlset的getTimestep设计的不好，应该在自动步长的时候去值为0，而不是  数组越界
* 最终还是解析文件时考虑不周，在没有STEP的时候应给它喂一个0，这样就不用写一堆判断了
* 5.自动步长更新有问题，最后步长竟然达到了6秒，先尝试给定步长看结果怎么样
* 6.程序崩溃，感受到备份的重要性

>4月15号 21:55更新
* 1.搞清楚为何计算的结果中，应力只有X方向的了，很简单计算应力的那一步函数，是材料类调用的，调用的竟然是1D，应该用三维的
>4月16号，7:00更新
* 1.今天主要任务是，通过将算例Variable的加速度分解施加，检验程序的可靠性，最终保证全约束无误
* 2.由于杆单元的限制，Y,Z方向的应力是计算不出来的，只有梁单元才行，但先不写，先保证杆无误。
* 3.再检查下解析文件是否无误，尤其是计算时的节点该有的约束都加上了没
* 4.删掉源.cpp文件中的其他实验算例
* 5.完蛋了，就首末加约束的节点有位移值，其他都是0
* 6.都是0的一个可能原因是，计算应力应变用的是单元，计算节点力后，把力分给了单元上的节点，单元结点和nodeList上的并不一定耦合，还是0

```cpp
void Rod_2::calculateStrain(double timestep, int integration_points){
	//sqrt(dx^2+dy^2+dz^2)
	double new_length = sqrt((leftNode->pos[0] - rightNode->pos[0])*(leftNode->pos[0] - rightNode->pos[0]) +
		(leftNode->pos[1] - rightNode->pos[1])*(leftNode->pos[2] - rightNode->pos[1]) +
		(leftNode->pos[2] - rightNode->pos[2])*(leftNode->pos[2] - rightNode->pos[2]));
	//ln strain
	dstrain[0][0] = log(1 + (new_length - initial_length) / initial_length) - strain[0][0];
	// Calculate also the new cross section area (assuming incompressible
	// material)
	cross_section_area = initial_cross_section_area*initial_length / new_length;
}//我尼玛，应变计算出错了，Dy错误
```

>4月16号，14:24更新
* 1.重新开始，，哎，，还好有一个能算出正确结果的程序，感觉16号上午，还有15号一天，完全没有思路的乱搞，搞得最后连最基本的都不能算了。。。。。
* 2. 睡一觉后，任务：对比之前改错的程序，找找原因，留下有用的，扔掉没用的;
* 继续思考多杆单元的问题
* 3.
 *		找到了一个关键问题，在Node的setInitialCondition方
 *	法中，竟然忘记了添加之前因为节点约束耦合无法编译而没能添加
 *	的设置初始边界条件，坑爹，这应该就是问题的一个重要源头
 *	但问题是，没有加，依然把单杆冲击问题算出来了。。无语
 * 重大问题已经找到了，是单元计算结点力时，的那个-号，引起的，见报告
 * 另外重新分析了下位移，感觉右节点的位移计算肯定有问题
 * 其实没问题
 * 总结下心路历程，无论遇到什么样的宕机型bug，都不要丧失理智和斗志,不妨睡一觉，哈哈
 * 很棘手的问题，调试时pos这个量明明在变化，有更新值，但就是出不出来，减值无语了

>4月19号 更新
* 1.更新二维数组与向量成绩的运算符重载函数
>5月22号 更新
* 1.单元刚度矩阵中，三维刚阵中的33-44-55项不对，多除了一个2、
* 2.另外，里面没有算20的应力

```cpp
elementMass = mass;
leftNode->mass += mass / 2.0;
//nodeTable[leftNode->number].mass = mass / 2.0;
rightNode->mass += mass / 2.0;
//nodeTable[rightNode->number].mass = mass / 2.0;
```
>5-23号更新
* 1.ARRAY2D中新增 二维矩阵与一维向量之积的操作符重载

>5月24号更新
* 1.今天解决SOLID6在文件解析，以及求解器不协调的问题
* 2.加入接触单元，完成实体单元间的接触模拟，检验程序的正确性
* 3.如果实体单元接触不能成功，就转入Java源码的编译试验中，验证源程序的可靠性
* 4.考虑通过刚度总装，的形式求解节点位移以及其他物理量，作为隐式求解的基础
* 5.加入刚体边界条件，其实就是一种耦合关系，将所有收到RIGID约束的节点耦合到主节点master里，使其他节点的运动与master相关;
* 为此，更改Node中约束的施加方案，添加两种applyACC/VEL，一种针对RIGID一种
	针对BC；；本来约束条件的施加应该在约束中以传入node的形式施加，但因为出现
	编译错误，只好先把这些内容放在Node中，等解决这个编译问题之后再迁移到约束中
* 6.类中声明一个方法后，如果不实现，就会爆出外部链接出错，所以声明的方法最好写个空方法体放在那儿。
* 7.做了一次非常彻底，非常大的改动，为了加入刚体约束，同时解决之前遇到的 Node约束矛盾的问题，把rigid中的一些方法重组到Node中，更新方法放到了
求解器中的时间步迭代中，同时为了能同时识别两种约束，加入了大量的判断语句，这样是不长久的，但目前先这样，随后再重构吧
*8.本来今天是要写完接触单元的，谁想一个刚性约束就折磨了我一天，，，无奈，明天再写三角接触元吧。
