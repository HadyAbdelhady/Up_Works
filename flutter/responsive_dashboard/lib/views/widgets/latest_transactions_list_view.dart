import 'package:flutter/material.dart';
import 'package:responsive_dashboard/models/user_info_model.dart';
import 'package:responsive_dashboard/utils/app_images.dart';
import 'package:responsive_dashboard/views/widgets/user_info_list_tile.dart';

class LatestTransactionsListView extends StatelessWidget {
  const LatestTransactionsListView({super.key});
  static const items = [
    UserInfoModel(
      image: Assets.imagesAvatar1,
      title: 'Josua Nunito',
      subTitle: 'Madraniadi20@gmail',
    ),
    UserInfoModel(
      image: Assets.imagesAvatar2,
      title: 'Madrani Andi',
      subTitle: 'Josh Nunito@gmail.com',
    ),
    UserInfoModel(
      image: Assets.imagesAvatar1,
      title: 'Madrani Andi',
      subTitle: 'Madraniadi20@gmail',
    )
  ];
  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      scrollDirection: Axis.horizontal,
      child: Row(
        children: items.map((e) => UserInfoListTile(userInfoModel: e)).toList(),
      ),
    );

    // return SizedBox(
    //   height: 70,
    //   child: ListView.builder(
    //     scrollDirection: Axis.horizontal,
    //     itemCount: items.length,
    //     itemBuilder: (BuildContext context, int index) {
    //       return IntrinsicWidth(
    //         child: UserInfoListTile(
    //           userInfoModel: items[index],
    //         ),
    //       );
    //     },
    //   ),
    // );
  }
}
