// JNI bindings for com.gtech.rtklib.RtkLib. Every call is serialized because RTKLIB keeps its
// processing state in globals; progress text is forwarded to a Java listener.

#include <android/log.h>
#include <jni.h>

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "ppk_core.h"

extern "C" {
#include "rtklib.h"
}

#define LOG_TAG "RtkLib"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

namespace {

std::mutex g_lock;

struct Listener {
  JNIEnv* env;
  jobject target;
  jmethodID onMessage;
};

void forwardMessage(void* context, const char* message) {
  auto* listener = static_cast<Listener*>(context);
  if (listener->target == nullptr || listener->onMessage == nullptr) return;
  JNIEnv* env = listener->env;
  jstring text = env->NewStringUTF(message);
  env->CallVoidMethod(listener->target, listener->onMessage, text);
  env->DeleteLocalRef(text);
  if (env->ExceptionCheck()) env->ExceptionClear();
}

/** Installs a Java listener as the RTKLIB message handler for one native call. */
class ListenerScope {
 public:
  ListenerScope(JNIEnv* env, jobject target) : listener_{env, target, nullptr} {
    if (target != nullptr) {
      jclass cls = env->GetObjectClass(target);
      listener_.onMessage = env->GetMethodID(cls, "onMessage", "(Ljava/lang/String;)V");
      env->DeleteLocalRef(cls);
    }
    ppk_reset_cancel();
    ppk_set_message_handler(forwardMessage, &listener_);
  }

  ~ListenerScope() { ppk_set_message_handler(nullptr, nullptr); }

 private:
  Listener listener_;
};

std::string toStdString(JNIEnv* env, jstring value) {
  if (value == nullptr) return "";
  const char* chars = env->GetStringUTFChars(value, nullptr);
  std::string result(chars);
  env->ReleaseStringUTFChars(value, chars);
  return result;
}

}  // namespace

extern "C" JNIEXPORT jstring JNICALL
Java_com_gtech_rtklib_RtkLib_nativeVersion(JNIEnv* env, jclass) {
  std::string version = std::string("RTKLIB ") + VER_RTKLIB + " " + PATCH_LEVEL;
  return env->NewStringUTF(version.c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_com_gtech_rtklib_RtkLib_nativeCancel(JNIEnv*, jclass) {
  ppk_cancel();
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_gtech_rtklib_RtkLib_nativeLastMessage(JNIEnv* env, jclass) {
  std::lock_guard<std::mutex> guard(g_lock);
  return env->NewStringUTF(ppk_last_message());
}

extern "C" JNIEXPORT jint JNICALL
Java_com_gtech_rtklib_RtkLib_nativeConvertUnicore(
    JNIEnv* env, jclass, jstring input, jstring obsOut, jstring navOut, jobject listener) {
  std::lock_guard<std::mutex> guard(g_lock);
  ListenerScope scope(env, listener);

  std::string inputPath = toStdString(env, input);
  std::string obsPath = toStdString(env, obsOut);
  std::string navPath = toStdString(env, navOut);
  LOGI("convert %s", inputPath.c_str());
  return ppk_convert_unicore(inputPath.c_str(), obsPath.c_str(), navPath.c_str());
}

/** options layout matches RtkLib.PpkOptions#toArray(). */
extern "C" JNIEXPORT jint JNICALL
Java_com_gtech_rtklib_RtkLib_nativePostPos(
    JNIEnv* env, jclass, jobjectArray inputs, jdoubleArray options, jstring outFile,
    jobject listener) {
  std::lock_guard<std::mutex> guard(g_lock);
  ListenerScope scope(env, listener);

  jsize n = env->GetArrayLength(inputs);
  std::vector<std::string> paths(n);
  std::vector<const char*> files(n);
  for (jsize i = 0; i < n; i++) {
    auto item = (jstring)env->GetObjectArrayElement(inputs, i);
    paths[i] = toStdString(env, item);
    env->DeleteLocalRef(item);
    files[i] = paths[i].c_str();
  }

  double values[19] = {0};
  values[15] = NAN;           // base antenna height: from the RINEX header unless given
  values[16] = TROPOPT_SAAS;  // troposphere model
  values[17] = EPHOPT_BRDC;   // satellite ephemeris
  values[18] = 1;             // solid earth tide on
  jsize count = env->GetArrayLength(options);
  env->GetDoubleArrayRegion(options, 0, count < 19 ? count : 19, values);

  ppk_options opt;
  opt.mode = (int)values[0];
  opt.soltype = (int)values[1];
  opt.nf = (int)values[2];
  opt.navsys = (int)values[3];
  opt.elmin_deg = values[4];
  opt.modear = (int)values[5];
  opt.glomodear = (int)values[6];
  opt.bdsmodear = (int)values[7];
  opt.ar_ratio = values[8];
  opt.refpos = (int)values[9];
  opt.ts = values[10];
  opt.te = values[11];
  opt.rb[0] = values[12];
  opt.rb[1] = values[13];
  opt.rb[2] = values[14];
  opt.base_ant_h = values[15];
  opt.tropopt = (int)values[16];
  opt.sateph = (int)values[17];
  opt.tidecorr = (int)values[18];

  std::string outPath = toStdString(env, outFile);
  LOGI("postpos n=%d out=%s", (int)n, outPath.c_str());
  return ppk_postpos(files.data(), (int)n, &opt, outPath.c_str());
}
