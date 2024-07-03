using System;
using System.Text;
using Unity.Collections.LowLevel.Unsafe;

namespace ImGuiNET
{
    // ImGui extra functionality related with Drag and Drop
    public static partial class ImGui
    {
        // TODO: review
        // can now pass refs with UnsafeUtility.AsRef

        public static unsafe void SetDragDropPayload<T>(string type, T data, ImGuiCond cond = 0)
        where T : unmanaged
        {
            void* ptr = UnsafeUtility.AddressOf(ref data);
            SetDragDropPayload(type, new IntPtr(ptr), (uint)UnsafeUtility.SizeOf<T>(), cond);
        }

        public static unsafe bool AcceptDragDropPayload<T>(string type, out T payload, ImGuiDragDropFlags flags = ImGuiDragDropFlags.None)
        where T : unmanaged
        {
            ImGuiPayload* pload = AcceptDragDropPayload(type, flags);
            payload = (pload != null) ? UnsafeUtility.ReadArrayElement<T>(pload->Data, 0) : default;
            return pload != null;
        }
    }
}
