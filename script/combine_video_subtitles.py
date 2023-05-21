from os.path import isfile
import subprocess
import requests
from pytube import YouTube


def DownloadYoutube(video_url, save_dir, file_name):

    # 创建YouTube对象
    yt = YouTube(video_url)

    # 获取视频的所有可用格式
    video_formats = yt.streams.filter(progressive=True).all()

    # 选择要下载的格式（这里选择第一个）
    video = video_formats[0]

    # 开始下载视频
    video.download(output_path=save_dir, filename='video.mp4')

    # 获取视频的所有字幕
    captions = yt.captions

    # 遍历所有字幕
    for caption in captions:
        # 打印字幕的语言和标识符
        print("语言:", caption.language)
        print("标识符:", caption.code)

        # 下载字幕文件
        caption.download(title_prefix=caption.code, output_path=save_dir)

    print("视频下载完成")


def CombineVideoSubtitles(videoFile, srtFile, outputFile):
    if not (isfile(videoFile) and videoFile.endswith(
        ('.avi', '.mp4')) and isfile(srtFile) and srtFile.endswith('.srt')):
        print('视频仅支持avi以及mp4,字幕仅支持srt格式')
    else:
        subprocess.run(
            f"ffmpeg -i {videoFile} -vf subtitles={srtFile} -y {outputFile}".
            split(' '))
        print('字幕合成完成')


if __name__ == '__main__':

    DownloadYoutube('https://www.youtube.com/watch?v=bHcJCp2Fyxs',
                    '/home/huangshuai/Downloads/')

    CombineVideoSubtitles('/home/huangshuai/Downloads/2.mp4',
                          '/home/huangshuai/Downloads/2.srt',
                          '/home/huangshuai/Downloads/2_out.mp4')
