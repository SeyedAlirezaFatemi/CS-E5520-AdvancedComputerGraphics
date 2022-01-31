
/*

NOTE: this is kind of a messy file; if you wish for more features
in the plotter, it might be easier to ask us for them than to try
and understand what's going on.





*/


// we'll be doing image manipulations and export using GDI+, a software graphics library built into windows.
#include <Windows.h>
#include <gdiplus.h>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <locale>
#include <codecvt>
#include <sstream>
#include <algorithm>
#include <map>

#pragma comment(lib, "Gdiplus.lib")

//straight from the GDI+ documentation
int GetEncoderClsid(const WCHAR* format, CLSID* pClsid);

struct DataPoint {
	unsigned t_build, t_trace, draw_i;
	std::string scene, state;
};

int main(int argc, char* argv[]) {

	using namespace Gdiplus;
	using namespace std;

	srand(381);

	cout.precision(2);

	vector<unsigned> cols = { /*0x5DA5DA,*/ 0xFAA43A, 0x60BD68, 0xF17CB0, 0xB2912F, 0xB276B2, 0xDECF3F, 0xF15854 };

	vector<string> args; // = { "plotter", "res_2.txt" };

	for (int i = 0; i < argc; ++i)
		args.push_back(argv[i]);

	GdiplusStartupInput input;
	input.GdiplusVersion = 1;

	ULONG_PTR gdi_token;
	GdiplusStartup(&gdi_token, &input, nullptr);

	auto width = 1000,
		height = 800;

	auto file = ifstream(args[1]);


	if (file.fail()) {
		cout << "Couldn't open input file '" << args[1] << "'!" << endl;
		system("pause");
		return -1;
	}

	auto image_path = args[1].substr(0, args[1].length() - 4) + ".png";

	struct time_sort_entry {
		unsigned total_time, total_build, total_trace, ray_count;
		string name;
	};

	map<string, vector<DataPoint>> data_base;
	map<string, time_sort_entry> total_time;

	int max_time = 0;

	string line, set;
	getline(file, line);
	while (getline(file, line) && line!="") {
		stringstream line_stream(line);
		DataPoint p;
		int ray_count;
		line_stream >> set >> p.scene >> p.state >> p.t_build >> p.t_trace >> ray_count;
		max_time = max(max_time, int(p.t_build + p.t_trace));
		data_base[set].push_back(p);
		if (total_time.count(set) == 0) {
			total_time[set].name = set;
			total_time[set].total_time = 0;
			total_time[set].total_build = 0;
			total_time[set].total_trace = 0;
			total_time[set].ray_count = 0;
		}
		total_time[set].total_time += p.t_build + p.t_trace;
		total_time[set].total_build += p.t_build;
		total_time[set].total_trace += p.t_trace;
		total_time[set].ray_count += ray_count;
	}

	vector<time_sort_entry> sorted_times;

	for (auto& data_set : total_time) {
		time_sort_entry tmp;
		tmp.name = data_set.first;
		tmp.total_time = data_set.second.total_time;
		tmp.total_build = data_set.second.total_build;
		tmp.total_trace = data_set.second.total_trace;
		sorted_times.push_back(tmp);
	}

	sort(sorted_times.begin(), sorted_times.end(), [&](const time_sort_entry& a, const time_sort_entry& b) -> bool {return a.total_time < b.total_time; });
	int j = 0;
	for (auto& a : sorted_times) {
		for (auto& b : data_base[a.name])
			b.draw_i = j;
		++j;
	}

	max_time = int(double(max_time)*1.2);

	int set_size = -1;

	for (auto& data_set : data_base) {
		if (set_size < 0)
			set_size = data_set.second.size();
		else if (set_size != data_set.second.size()) {
			set_size = -1;
			break;
		}
	}

	int set_count = data_base.size();

	vector<Color> colors(set_count);
	for (int i = 0; i < set_count; ++i){
		colors[i].SetFromCOLORREF(cols[i % cols.size()]);
	}

	if (set_size < 0) {
		cout << "All sets must be of same length!" << endl;
		system("pause");
	} else {

		auto& img = Bitmap(width, height, PixelFormat32bppARGB);

		auto gfx = Graphics::FromImage(&img);
		//ensure white background
		{
			auto& background_brush = SolidBrush(Color(255, 255, 255, 255));
			auto& background_rect = Rect(0, 0, width, height);

			gfx->FillRectangle(&background_brush, background_rect);
		}
		width -= 300;
		height -= 100;

		auto& f = Font(L"Segoe UI Light", 12, FontStyleRegular);
		auto& title_font = Font(L"Segoe UI", 24, FontStyleBold);

		auto& black_brush = SolidBrush(Color(255, 0, 0, 0));

		auto& arrow_pen = Pen(&black_brush, 1.0);
		arrow_pen.SetStartCap(LineCapArrowAnchor);
		
		gfx->SetSmoothingMode(SmoothingModeHighQuality);
		
		gfx->SetTextRenderingHint(TextRenderingHintAntiAlias);

		StringFormat vertical_format;
		vertical_format.SetFormatFlags(StringFormatFlags::StringFormatFlagsDirectionVertical);
		StringFormat right_format;
		right_format.SetAlignment(StringAlignment::StringAlignmentFar);
		StringFormat center_format;
		center_format.SetAlignment(StringAlignment::StringAlignmentCenter);

		gfx->DrawLine(&arrow_pen, PointF(.1f*width, .1f*height), PointF(.1f*width, .9f*height));
		gfx->DrawLine(&arrow_pen, PointF(.9f*width, .9f*height), PointF(.1f*width, .9f*height));

		bool is_first = true;

		int set_width = int(.8f * width / float(set_size));
		int col_width = int(1+set_width / float(set_count + 1));

		std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

		int last_slash = args[1].find_last_of("/\\")+1;

		gfx->DrawString(converter.from_bytes(args[1].substr(last_slash,(args[1].length()-4-last_slash))).c_str(), -1, &title_font, PointF(.5f*width, .05f*height), &center_format, &black_brush);

		int interval = 100 * max(1,int(max_time / 1000));

		for (int i = 0; i < max_time; i += interval) {
			int line_height = int(float(int(.9f*height)) - .8*height*float(i) / float(max_time));
			auto& pen = Pen(Color(255, 0, 0, 0), 1.0f);
			gfx->DrawLine(&pen, Point(int(.1f*width - 4), line_height), Point(int(.1f*width), line_height));
			gfx->DrawString(converter.from_bytes(to_string(i)).c_str(), -1, &f, PointF(.1f*width - 8, float(line_height - 12)), &right_format, &black_brush);
		}
		{
			auto& background_brush = SolidBrush(Color(255, 220, 215, 200));
			auto& background_rect = Rect(int(width*.9f), int(.1f*height - 2), 300, set_count * 3 * 26 + 4);

			gfx->FillRectangle(&background_brush, background_rect);
			gfx->DrawRectangle(&arrow_pen, background_rect);
		}
		for (auto& data_set : data_base) {
			int set_i = data_set.second[0].draw_i;
			int point_i = 0;
			auto& brush = SolidBrush(colors[set_i]);
			auto& brush2 = SolidBrush(Color(BYTE(colors[set_i].GetR()*.8), BYTE(colors[set_i].GetG()*.8), BYTE(colors[set_i].GetB()*.8)));
			auto& pen = Pen(Color(255, 0, 0, 0), 1.0f);

			auto legend_rect1 = Rect(int(width*.9f) + 6, int(.1f*height) + 6 + 26*3 * set_i, 10, 14);
			auto legend_rect2 = Rect(int(width*.9f) + 16, int(.1f*height) + 6 + 26*3 * set_i, 10, 14);

			gfx->FillRectangle(&brush, legend_rect1);
			gfx->DrawRectangle(&pen, legend_rect1);
			gfx->FillRectangle(&brush2, legend_rect2);
			gfx->DrawRectangle(&pen, legend_rect2);
			ostringstream value;
			value << setprecision(3) << (1.0f / 1000.0f)*float(total_time[data_set.first].ray_count) / float(total_time[data_set.first].total_trace) << " MRay/s, " << (1.0f / 1000.0f)*float(total_time[data_set.first].ray_count) / float(total_time[data_set.first].total_time);
			gfx->DrawString(converter.from_bytes(data_set.first+"("+to_string(total_time[data_set.first].total_time)+"ms)").c_str(), -1, &f, PointF(width*.9f + 30, .1f*height +2 + 26 *3* set_i), &black_brush);
			gfx->DrawString(converter.from_bytes("build " + to_string(total_time[data_set.first].total_build) + "ms, trace " + to_string(total_time[data_set.first].total_trace) + "ms").c_str(), -1, &f, PointF(width*.9f + 30, .1f*height + 28 + 26 * 3 * set_i), &black_brush);
			gfx->DrawString(converter.from_bytes(value.str() + " incl. build").c_str(), -1, &f, PointF(width*.9f + 30, .1f*height + 54 + 26 * 3 * set_i), &black_brush);
			for (auto& data_point : data_set.second) {
				int pos_x = int(.1f*width) + set_width*point_i + col_width*data_point.draw_i;
				int col_height = int(.8*height*float(data_point.t_build + data_point.t_trace) / float(max_time));
				int build_height = int(.8*height*float(data_point.t_build) / float(max_time));
				auto r = Rect(Point(pos_x, int(.9f*height) - col_height), Size(col_width, col_height));
				auto r2 = Rect(Point(pos_x, int(.9f*height) - build_height), Size(col_width, build_height));
				gfx->FillRectangle(&brush, r);
				gfx->DrawRectangle(&pen, r);
				gfx->FillRectangle(&brush2, r2);
				gfx->DrawRectangle(&pen, r2);

				if (is_first) {

					gfx->DrawString(converter.from_bytes(data_point.state).c_str(), -1, &f, PointF(.1f*width+set_width*point_i+col_width*set_count*.5f-10, .9f*height), &vertical_format, &black_brush);

				}

				++point_i;
			}
			is_first = false;
		}

		CLSID png_encoder;
		GetEncoderClsid(L"image/png", &png_encoder);


		img.Save(converter.from_bytes(image_path).c_str(), &png_encoder);
	}

	GdiplusShutdown(gdi_token);
	
	ShellExecuteA(0, 0, ("\""+image_path+"\"").c_str(), 0, 0, SW_SHOW);
	return 0;
}

// from http://msdn.microsoft.com/en-us/library/windows/desktop/ms533843(v=vs.85).aspx
int GetEncoderClsid(const WCHAR* format, CLSID* pClsid)
{

	UINT  num = 0;          // number of image encoders
	UINT  size = 0;         // size of the image encoder array in bytes

	Gdiplus::ImageCodecInfo* pImageCodecInfo = NULL;

	Gdiplus::GetImageEncodersSize(&num, &size);
	if (size == 0)
		return -1;  // Failure

	pImageCodecInfo = (Gdiplus::ImageCodecInfo*)(malloc(size));
	if (pImageCodecInfo == NULL)
		return -1;  // Failure

	Gdiplus::GetImageEncoders(num, size, pImageCodecInfo);

	for (UINT j = 0; j < num; ++j)
	{
		if (wcscmp(pImageCodecInfo[j].MimeType, format) == 0)
		{
			*pClsid = pImageCodecInfo[j].Clsid;
			free(pImageCodecInfo);
			return j;  // Success
		}
	}

	free(pImageCodecInfo);
	return -1;  // Failure
}