#include <bits/stdc++.h>
using namespace std;

string int2str(int n)
{
	string s="";
	if(n==0) return string("0");
	while(n){
		s+=(char('0'+(n%10)));
		n/=10;
	}
	reverse(s.begin(),s.end());
	return s;
}

int str2int(string s){
	int len=s.length();
	int n=0;
	for(int i=0;i<len;i++){
		n=n*10+(s[i]-'0');
	}
	return n;
}

int main()
{
	string filename="ex_ROU.rou.xml";
	//cin>>filename;
	fstream fio(filename.c_str());
	ofstream fout("right.rou.xml");
	
	string line;
	
	for(int i=0;i<20;i++){
		getline(fio,line);
		fout<<line<<endl;
	}
	
	int flows[20]={0};
	for(int j=0;j<20;j++)
		flows[j]=100*j;	
		
	for(int i=1;getline(fio,line);i++){
		//cout<<line<<endl;
		if(i%3!=1){
			fout<<line<<endl;
			continue;
		} 
		int p=line.find("flow");
		int first=line.find("\"");
		int second=line.find("\"",first+1);
		cout<<first<<" "<<second<<endl;
		cout<<p<<endl;
		if(p==-1){
			fout<<line<<endl;
			continue;
		}
		//int bias=(line[p+8]=='\"')?8:7;
		string s=line.substr(p+4,second-first-5);
		cout<<s<<endl;
		int dotpos=s.find(".");
		string f=s.substr(0,dotpos);
		string c=s.substr(dotpos+1,s.size());
		cout<<f<<" "<<c<<endl;
		int carNum=str2int(f)*100+str2int(c);
		cout<<str2int(f)<<" "<<str2int(c)<<" "<<carNum<<endl;
					
		//cout<<flow<<endl;
		
		line.replace(p,second-first-1,int2str(carNum));
		cout<<line<<endl;
		fout<<line<<endl;
		
	}
} 
