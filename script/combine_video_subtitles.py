import os
import shutil
import subprocess
from multiprocessing.dummy import Pool as ThreadPool
from pytube import YouTube, Playlist


def CombineVideoSubtitles(video_path, srt_path=None):
    output_file = os.path.join(os.path.dirname(video_path),
                               "combine_" + os.path.basename(video_path))

    if srt_path == None:
        shutil.copy(video_path, output_file)
        return

    print("start process combine ", video_path, srt_path)
    if not (os.path.isfile(video_path) and video_path.endswith(
        ('.avi', '.mp4')) and os.path.isfile(srt_path)
            and srt_path.endswith('.srt')):
        print('视频仅支持avi以及mp4,字幕仅支持srt格式')
    else:

        subprocess.run([
            'ffmpeg', '-i', video_path, '-vf', f'subtitles= {srt_path}', '-y',
            output_file
        ])
        print('字幕合成完成')


def DownLoadYoutubeFile(info):
    video_url, save_dir, list_index = info

    # 创建YouTube对象
    yt = YouTube(video_url)

    # 获取视频的所有可用格式
    video_formats = yt.streams.filter(progressive=True)

    # 选择要下载的格式（这里选择第一个）
    video = video_formats[-1]

    # 开始下载视频
    file_name_prefix = f"{list_index}_" if list_index != None else ""
    video_path = video.download(output_path=save_dir,
                                filename=file_name_prefix +
                                video.default_filename)

    # 获取视频的所有字幕
    captions = yt.captions

    # 打印字幕的语言代码和名称
    for caption in captions:
        print(caption.code, caption.name)

    if "zh-TW" in captions:
        # 选择你想要的字幕语言，比如中文
        caption = captions["zh-TW"]

        # 生成srt格式的字幕
        # 如果发生coredump，参考https://cloud.tencent.com/developer/ask/sof/284174
        srt_captions = caption.generate_srt_captions()

        # 下载字幕文件到当前目录，可以用output_path参数指定其他路径
        caption_path = caption.download(file_name_prefix + yt.title + '.srt',
                                        output_path=save_dir)
        print("视频下载完成")

        CombineVideoSubtitles(video_path, caption_path)
    else:
        CombineVideoSubtitles(video_path)


def DownloadYoutubeList(list_url, save_dir):

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # 设置环境变量
    os.environ["HTTP_PROXY"] = "http://your_proxy_server:your_proxy_port"
    os.environ["HTTPS_PROXY"] = "http://your_proxy_server:your_proxy_port"

    # 获取youtube list
    pl = Playlist(list_url)

    infos = []
    for i in range(len(pl.video_urls)):
        infos.append((pl.video_urls[i], save_dir, i + 1))

    # batch download
    pool = ThreadPool(10)
    pool.map(DownLoadYoutubeFile, infos)
    pool.close()
    pool.join()


if __name__ == '__main__':

    DownloadYoutubeList(
        'https://www.youtube.com/playlist?list=PLJV_el3uVTsMhtt7_Y6sgTHGHp1Vb2P2J',
        os.path.join(os.path.expanduser('~'), '/Desktop/hanson'))
